#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, StackResources};
use embassy_sync::channel;
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use embedded_svc::wifi::{Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiError, WifiStaDevice};
use esp_wifi::EspWifiInitFor;
use fixed::types::I20F12;
use hal::adc::Attenuation;
use hal::clock::{Clocks, CpuClock};
use hal::dma::DmaPriority;
use hal::gdma::{self, Gdma};
use hal::gpio::{GpioPin, Unknown};
use hal::rtc_cntl::sleep::TimerWakeupSource;
use hal::spi::master::dma::WithDmaSpi2;
use hal::spi::{FullDuplexMode, SpiMode};
use hal::systimer::SystemTimer;
use hal::{clock::ClockControl, peripherals::Peripherals, timer::TimerGroup, Rtc};
use hal::{prelude::*, Rng, IO};
use static_cell::StaticCell;

use temperature_firmware::{
    get_time_from_influx, singleton, write_measurements, GetTimeError, Measurement, Temperature,
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");
const MS_PER_MEASUREMENT: u64 = 15_000;

type Channel<T> =
    channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;
type Receiver<T> =
    channel::Receiver<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;
type Sender<T> =
    channel::Sender<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;

type MeasurementBuffer = ArrayVec<Measurement, 64>;

#[ram(rtc_fast, uninitialized)]
static mut MEASUREMENT_BUFFER: MeasurementBuffer = ArrayVec::new_const();

// A static bool, to put a safe API around MEASUREMENT_BUFFER.
static INITED: StaticCell<()> = StaticCell::new();

fn get_measurement_buffer() -> &'static mut MeasurementBuffer {
    // This will panic if it was already inited, making the following line safe
    // because the mut reference will be unique.
    INITED.init(());

    let cause = hal::reset::get_wakeup_cause();
    if !matches!(cause, hal::reset::SleepSource::Timer) {
        println!("wakeup caused by {:?}, initializing measurements", cause);
        unsafe {
            MEASUREMENT_BUFFER = ArrayVec::new();
        }
    }

    unsafe { &mut MEASUREMENT_BUFFER }
}

struct Led {
    spi: hal::spi::master::dma::SpiDma<
        'static,
        hal::peripherals::SPI2,
        gdma::Channel0,
        FullDuplexMode,
    >,
    buf: [u8; 48],
}

impl Led {
    fn new(
        dma: hal::peripherals::DMA,
        spi: hal::peripherals::SPI2,
        pin: GpioPin<Unknown, 7>,
        clocks: &Clocks,
    ) -> Self {
        let descriptors = singleton!([0u32; 8 * 3], [u32; 8 * 3]);
        hal::interrupt::enable(
            hal::peripherals::Interrupt::DMA_CH0,
            hal::interrupt::Priority::Priority1,
        )
        .unwrap();

        let dma = Gdma::new(dma);
        let dma_channel =
            dma.channel0
                .configure(false, descriptors, &mut [], DmaPriority::Priority0);

        let spi =
            hal::spi::master::Spi::new_mosi_only(spi, pin, 3333u32.kHz(), SpiMode::Mode0, clocks)
                .with_dma(dma_channel);
        Self { spi, buf: [0; 48] }
    }

    async fn color(&mut self, r: u8, g: u8, b: u8) {
        self.write_color(r, g, b);
        let _ = SpiBus::write(&mut self.spi, &self.buf).await;
    }

    fn write_color(&mut self, r: u8, g: u8, b: u8) {
        Self::write_byte(&mut self.buf[0..4], g);
        Self::write_byte(&mut self.buf[4..8], r);
        Self::write_byte(&mut self.buf[8..12], b);
    }

    fn write_byte(buf: &mut [u8], mut b: u8) {
        let patterns = [0b1000_1000, 0b1000_1110, 0b1110_1000, 0b1110_1110];
        for out in buf {
            let bits = (b & 0b1100_0000) >> 6;
            *out = patterns[bits as usize];
            b <<= 2;
        }
    }
}

#[main]
async fn main(spawner: Spawner) -> ! {
    let measurements = get_measurement_buffer();
    println!("buffer has {} measurements", measurements.len());

    if measurements.is_full() {
        transmit(spawner, measurements).await;
    } else {
        measure(measurements).await;
    }

    // The unreachable!() shuts up a rust-analyzer warning (because ra doesn't
    // realize that executor.run diverges), and the `allow` shuts up a rustc warning
    // (because it does realize the executor.run diverges).
    #[allow(unreachable_code)]
    {
        unreachable!();
    }
}

async fn connect(
    mut controller: WifiController<'static>,
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    led: Sender<LedState>,
) -> Result<(), WifiError> {
    let client_config = Configuration::Client(embedded_svc::wifi::ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    controller.set_configuration(&client_config)?;
    println!("Starting controller...");
    led.send(LedState::Other).await;
    // TODO: add a timeout
    controller.start().await?;

    led.send(LedState::WaitingForNetwork).await;
    println!("Connecting...");
    // TODO: add a timeout
    controller.connect().await?;

    led.send(LedState::Other).await;
    println!("Waiting to get IP address...");
    for _ in 0..40 {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            return Ok(());
        }
        led.send(LedState::WaitingForNetwork).await;
        Timer::after(Duration::from_millis(250)).await;
        led.send(LedState::Other).await;
        Timer::after(Duration::from_millis(250)).await;
    }

    // TODO: better error
    Err(WifiError::Disconnected)
}

enum SendMeasurementError {
    Connect(embassy_net::tcp::ConnectError),
    Get(GetTimeError),
    Send(temperature_firmware::SendMeasurementError),
    Dns(embassy_net::dns::Error),
    NoAddress,
}

impl From<embassy_net::dns::Error> for SendMeasurementError {
    fn from(v: embassy_net::dns::Error) -> Self {
        Self::Dns(v)
    }
}

impl From<embassy_net::tcp::ConnectError> for SendMeasurementError {
    fn from(v: embassy_net::tcp::ConnectError) -> Self {
        Self::Connect(v)
    }
}

impl From<temperature_firmware::SendMeasurementError> for SendMeasurementError {
    fn from(v: temperature_firmware::SendMeasurementError) -> Self {
        Self::Send(v)
    }
}

impl From<GetTimeError> for SendMeasurementError {
    fn from(v: GetTimeError) -> Self {
        Self::Get(v)
    }
}

async fn send_measurements(
    stack: &Stack<WifiDevice<'static, WifiStaDevice>>,
    rtc: &Rtc<'_>,
    measurements: &MeasurementBuffer,
) -> Result<(), SendMeasurementError> {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    let address = stack.dns_query(INFLUX_SERVER, DnsQueryType::A).await?;
    let address = *address.first().ok_or(SendMeasurementError::NoAddress)?;

    let remote_endpoint = (address, 8086);
    // TODO: can we reuse the same socket and/or connection? It isn't super-trivial
    // because without doing a one-shot connection we don't know when to stop trying
    // to read from the HTTP response.
    let date = {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        socket.connect(remote_endpoint).await?;

        let date = get_time_from_influx(&mut socket).await?;
        socket.close();
        date
    };

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    socket.connect(remote_endpoint).await?;
    // TODO: This is super bizarre. On deep sleep, the rtc_ms field in the
    // first measurement always gets zeroed. We fix it by always ignoring the
    // first measurement. (Below this loop, we stick in a dummy first measurement
    // so that we aren't actually skipping any data.)
    write_measurements(
        &mut socket,
        SENSOR_ID,
        date.assume_utc(),
        rtc,
        measurements.iter().skip(1),
    )
    .await?;

    Ok(())
}

async fn transmit(spawner: Spawner, measurements: &'static mut MeasurementBuffer) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    hal::embassy::init(&clocks, timer_group0.timer0);

    let led = Led::new(peripherals.DMA, peripherals.SPI2, io.pins.gpio7, &clocks);

    let led_channel = singleton!(Channel::new(), Channel<LedState>);

    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    let stack_resources = singleton!(StackResources::new(), StackResources::<3>);
    let dhcp_config = embassy_net::Config::dhcpv4(Default::default());
    let stack = singleton!(
        Stack::new(wifi_interface, dhcp_config, stack_resources, 1234,),
        Stack<WifiDevice<'static, WifiStaDevice>>
    );

    spawner.must_spawn(temperature_firmware::net_task(stack));
    spawner.must_spawn(led_task(led, led_channel.receiver()));
    led_channel.send(LedState::WaitingForNetwork).await;

    if connect(controller, stack, led_channel.sender())
        .await
        .is_err()
    {
        led_channel.send(LedState::Idle).await;
        Timer::after(Duration::from_millis(100)).await;
        let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(100));
        let mut delay = hal::Delay::new(&clocks);
        rtc.sleep_deep(&[&mut wake], &mut delay);
    }

    led_channel.send(LedState::Sending).await;

    let result = send_measurements(stack, &rtc, &*measurements).await;
    if result.is_ok() {
        measurements.clear();
        measurements.push(Measurement {
            temperature: Temperature {
                deg: I20F12::from_num(77),
            },
            battery: None,
            rtc_ms: 77,
        });
    }

    led_channel.send(LedState::Idle).await;
    Timer::after(Duration::from_millis(100)).await;
    let rtc_now = rtc.get_time_ms();
    let wakeup_time = rtc_now + MS_PER_MEASUREMENT;
    let quantized_wakeup_time = wakeup_time - wakeup_time % MS_PER_MEASUREMENT;
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(
        quantized_wakeup_time.saturating_sub(rtc_now),
    ));
    let mut delay = hal::Delay::new(&clocks);
    rtc.sleep_deep(&[&mut wake], &mut delay);
}

async fn measure(measurements: &'static mut MeasurementBuffer) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let analog = peripherals.APB_SARADC.split();

    let mut builder = temperature_firmware::analog::AdcBuilder::default();
    #[cfg(feature = "battery")]
    let mut battery_sensor = builder.add_activated_pin(
        io.pins.gpio3.into_analog(),
        io.pins.gpio2.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );
    let mut temperature_sensor = builder.add_activated_pin(
        io.pins.gpio4.into_analog(),
        io.pins.gpio0.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );
    let mut adc = builder.build(analog.adc1);

    hal::embassy::init(&clocks, timer_group0.timer0);

    let mut led = Led::new(peripherals.DMA, peripherals.SPI2, io.pins.gpio7, &clocks);
    led.color(1, 1, 1).await;

    #[cfg(feature = "battery")]
    let battery = Some(battery_sensor.read_voltage(&mut adc).await * 4);
    #[cfg(not(feature = "battery"))]
    let battery = None;

    let temperature =
        Temperature::from_tmp36_voltage(temperature_sensor.read_voltage(&mut adc).await);
    let rtc_ms = rtc.get_time_ms();
    measurements.push(Measurement {
        rtc_ms,
        battery,
        temperature,
    });

    led.color(0, 0, 0).await;
    let sleep_ms = if measurements.is_full() {
        0
    } else {
        let rtc_now = rtc.get_time_ms();
        let wakeup_time = rtc_now + MS_PER_MEASUREMENT;
        let quantized_wakeup_time = wakeup_time - wakeup_time % MS_PER_MEASUREMENT;
        quantized_wakeup_time.saturating_sub(rtc_now)
    };
    println!(
        "now is {}, planning to sleep for {} ms",
        rtc.get_time_ms(),
        sleep_ms
    );
    println!(
        "buf has {} measurements, last time is {}",
        measurements.len(),
        measurements.last().unwrap().rtc_ms
    );
    Timer::after(Duration::from_millis(100)).await;

    let mut delay = hal::Delay::new(&clocks);
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(sleep_ms));
    rtc.sleep_deep(&[&mut wake], &mut delay);
}

enum LedState {
    WaitingForNetwork,
    Idle,
    Sending,
    Other,
}

#[embassy_executor::task]
async fn led_task(mut led: Led, state: Receiver<LedState>) {
    led.color(0, 0, 0).await;
    loop {
        let state = state.receive().await;

        match state {
            LedState::WaitingForNetwork => led.color(8, 0, 0).await,
            LedState::Idle => led.color(0, 0, 0).await,
            LedState::Sending => led.color(0, 0, 2).await,
            LedState::Other => led.color(0, 1, 2).await,
        }
    }
}
