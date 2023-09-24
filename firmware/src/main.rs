#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use core::fmt::Write;

use arrayvec::ArrayVec;
use embassy_executor::_export::StaticCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_sync::channel;
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use embedded_svc::wifi::{Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiDevice, WifiMode};
use esp_wifi::EspWifiInitFor;
use hal::adc::{AdcCalLine, AdcConfig, AdcPin, Attenuation, ADC, ADC1};
use hal::clock::{Clocks, CpuClock};
use hal::dma::DmaPriority;
use hal::gdma::{self, Gdma};
use hal::gpio::{Analog, GpioPin, Output, PushPull, Unknown};
use hal::rtc_cntl::sleep::TimerWakeupSource;
use hal::spi::{FullDuplexMode, SpiMode};
use hal::system::PeripheralClockControl;
use hal::systimer::SystemTimer;
use hal::{clock::ClockControl, peripherals::Peripherals, timer::TimerGroup, Rtc};
use hal::{prelude::*, Rng, IO};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const MS_PER_MEASUREMENT: u64 = 5_000;

type Channel<T> =
    channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;
type Receiver<T> =
    channel::Receiver<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;

type MeasurementBuffer = ArrayVec<Measurement, 5>;

#[ram(rtc_fast, uninitialized)]
static mut MEASUREMENT_BUFFER: MeasurementBuffer = ArrayVec::new_const();

// A static bool, to put a safe API around MEASUREMENT_BUFFER.
static INITED: StaticCell<()> = StaticCell::new();

struct Measurement {
    rtc_ms: u64,
    temperature_millidegs: i32,
    battery_millivolts: u32,
}

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

struct SensorPin<const P: u8> {
    atten: Attenuation,
    pin: AdcPin<GpioPin<Analog, P>, ADC1, AdcCalLine<ADC1>>,
}

struct Writer {
    buf: [u8; 256],
    cursor: usize,
}

impl Default for Writer {
    fn default() -> Self {
        Self {
            buf: [0; 256],
            cursor: 0,
        }
    }
}

impl Writer {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.cursor]).unwrap()
    }

    fn clear(&mut self) {
        self.cursor = 0;
    }

    fn len(&self) -> usize {
        self.cursor
    }
}

impl core::fmt::Write for Writer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // TODO: error on buffer overflow
        let len = s.len();
        self.buf[self.cursor..(self.cursor + len)].copy_from_slice(s.as_bytes());
        self.cursor += s.len();
        Ok(())
    }
}

macro_rules! singleton {
    ($val:expr, $typ:ty) => {{
        static STATIC_CELL: StaticCell<$typ> = StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

struct Led {
    spi: hal::spi::dma::SpiDma<'static, hal::peripherals::SPI2, gdma::Channel0, FullDuplexMode>,
    buf: [u8; 48],
}

impl Led {
    fn new(
        dma: hal::peripherals::DMA,
        spi: hal::peripherals::SPI2,
        pin: GpioPin<Unknown, 7>,
        clocks: &Clocks,
        cc: &mut PeripheralClockControl,
    ) -> Self {
        let descriptors = singleton!([0u32; 8 * 3], [u32; 8 * 3]);
        hal::interrupt::enable(
            hal::peripherals::Interrupt::DMA_CH0,
            hal::interrupt::Priority::Priority1,
        )
        .unwrap();

        let dma = Gdma::new(dma, cc);
        let dma_channel =
            dma.channel0
                .configure(false, descriptors, &mut [], DmaPriority::Priority0);

        let spi = hal::spi::Spi::new_mosi_only(spi, pin, 3333u32.kHz(), SpiMode::Mode0, cc, clocks)
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

async fn read_raw<const P: u8>(
    adc: &mut ADC<'_, ADC1>,
    pin: &mut AdcPin<GpioPin<Analog, P>, ADC1, AdcCalLine<ADC1>>,
) -> u16
where
    GpioPin<Analog, P>: embedded_hal::adc::Channel<ADC1, ID = u8>,
{
    loop {
        let pin_value: nb::Result<u16, _> = adc.read(pin);
        if let Ok(val) = pin_value {
            println!("read {}", val);
            return val;
        } else {
            // adc.read never returns an error other than WouldBlock, so we
            // can assume it's WouldBlock here.
            Timer::after(Duration::from_micros(50)).await;
        }
    }
}

async fn read_uv<const P: u8>(adc: &mut ADC<'_, ADC1>, pin: &mut SensorPin<P>) -> i32
where
    GpioPin<Analog, P>: embedded_hal::adc::Channel<ADC1, ID = u8>,
{
    let mut total = read_raw(adc, &mut pin.pin).await;
    let mut max = total;
    let mut min = total;
    for _ in 0..9 {
        let reading = read_raw(adc, &mut pin.pin).await;
        max = max.max(reading);
        min = min.min(reading);
        total += reading;
    }

    total -= min;
    total -= max;
    total as i32 * pin.atten.ref_mv() as i32 / 1024 * 1000 / 4 / 8
}

struct Sensors {
    adc: ADC<'static, ADC1>,
    battery: SensorPin<1>,
    battery_activate: GpioPin<Output<PushPull>, 0>,
    temperature: SensorPin<3>,
}

impl Sensors {
    fn new(
        adc: ADC1,
        battery: GpioPin<Unknown, 1>,
        battery_activate: GpioPin<Unknown, 0>,
        temperature: GpioPin<Unknown, 3>,
        pcc: &mut PeripheralClockControl,
    ) -> Self {
        let atten = Attenuation::Attenuation2p5dB;
        let mut adc_config = AdcConfig::new();
        let temp_pin = SensorPin {
            pin: adc_config
                .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(temperature.into_analog(), atten),
            atten,
        };
        let battery_pin = SensorPin {
            pin: adc_config
                .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(battery.into_analog(), atten),
            atten,
        };
        let battery_activate = battery_activate.into_push_pull_output();
        let adc1: ADC<'static, ADC1> = ADC::<ADC1>::adc(pcc, adc, adc_config).unwrap();
        Sensors {
            temperature: temp_pin,
            battery: battery_pin,
            battery_activate,
            adc: adc1,
        }
    }

    /// Reads the current temperature in milli-degrees
    async fn read_temperature(&mut self) -> i32 {
        let uv = read_uv(&mut self.adc, &mut self.temperature).await;
        println!("temp {} uV", (uv - 500_000) / 10);
        (uv - 500_000) / 10
    }

    /// Returns the current battery voltage, in millivolts.
    async fn read_battery(&mut self) -> i32 {
        let _ = self.battery_activate.set_high();
        Timer::after(Duration::from_millis(5)).await;
        let uv = read_uv(&mut self.adc, &mut self.battery).await;
        let _ = self.battery_activate.set_low();
        println!("batt {} uV", uv);
        // There's a 10k and a 68k resistor involved, so we need to rescale the voltage back. TODO: explain better
        uv * 78 / 10 / 1000
    }
}

#[entry]
fn main() -> ! {
    let executor = singleton!(Executor::new(), Executor);
    let measurements = get_measurement_buffer();
    println!("buffer has {} measurements", measurements.len());

    executor.run(|spawner| {
        if measurements.is_full() {
            spawner.must_spawn(transmit(spawner, measurements));
        } else {
            spawner.must_spawn(measure(measurements));
        }
    });

    // The unreachable!() shuts up a rust-analyzer warning (because ra doesn't
    // realize that executor.run diverges), and the `allow` shuts up a rustc warning
    // (because it does realize the executor.run diverges).
    #[allow(unreachable_code)]
    {
        unreachable!();
    }
}

#[embassy_executor::task]
async fn transmit(spawner: Spawner, measurements: &'static mut MeasurementBuffer) {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    // FIXME: do I have to do this *before* spawning an async task?
    // I think not, based on the embassy::main macro.
    hal::embassy::init(&clocks, timer_group0.timer0);

    let led = Led::new(
        peripherals.DMA,
        peripherals.SPI2,
        io.pins.gpio7,
        &clocks,
        &mut system.peripheral_clock_control,
    );

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

    let wifi = peripherals.RADIO.split().0;
    let (wifi_interface, mut controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiMode::Sta).unwrap();
    let stack_resources = singleton!(StackResources::new(), StackResources::<3>);
    let dhcp_config = embassy_net::Config::dhcpv4(Default::default());
    let stack = singleton!(
        Stack::new(wifi_interface, dhcp_config, stack_resources, 1234,),
        Stack<WifiDevice<'static>>
    );

    spawner.must_spawn(net_task(stack));
    spawner.must_spawn(led_task(led, led_channel.receiver()));
    led_channel.send(LedState::WaitingForNetwork).await;

    let client_config = Configuration::Client(embedded_svc::wifi::ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    println!("Starting controller...");
    controller.start().await.unwrap();

    println!("Connecting...");
    match controller.connect().await {
        Ok(_) => println!("Wifi connected!"),
        Err(e) => {
            println!("Failed to connect to wifi: {:?}", e);
        }
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    led_channel.send(LedState::Sending).await;
    let mut write = Writer::default();
    let mut req = Writer::default();
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let remote_endpoint = (Ipv4Address::new(192, 168, 86, 118), 3000);
    let r = socket.connect(remote_endpoint).await;
    if let Err(e) = r {
        println!("connect error: {:?}", e);
    }

    for meas in &*measurements {
        write.clear();
        req.clear();

        let now = rtc.get_time_ms();
        let ms_ago = now.saturating_sub(meas.rtc_ms);

        // FIXME: this is wrong for negative temperatures
        write!(
            &mut write,
            "{{ \"sensor_id\": {}, \"temperature\": {}.{:03}, \"battery\": {}, \"ms_ago\": {} }}",
            SENSOR_ID,
            meas.temperature_millidegs / 1_000,
            meas.temperature_millidegs % 1_000,
            meas.battery_millivolts,
            ms_ago
        )
        .unwrap();

        write!(&mut req, "POST /collect HTTP/1.0\r\n").unwrap();
        write!(&mut req, "Connection: Close\r\n").unwrap();
        write!(&mut req, "Content-type: application/json\r\n").unwrap();
        write!(&mut req, "Content-length: {}\r\n", write.len()).unwrap();
        write!(&mut req, "\r\n").unwrap();
        write!(&mut req, "{}", write.as_str()).unwrap();

        println!("sending request {}", req.as_str());

        use embedded_io::asynch::Write;
        let r = socket.write_all(req.as_str().as_bytes()).await;
        if let Err(e) = r {
            println!("write error: {:?}", e);
            // FIXME: this aborts. Try to recover instead
            break;
        }

        let mut buf = [0; 256];
        let n = match socket.read(&mut buf).await {
            Ok(0) => {
                println!("read EOF");
                break;
            }
            Ok(n) => n,
            Err(e) => {
                println!("read error: {:?}", e);
                break;
            }
        };
        println!("{}", core::str::from_utf8(&buf[..n]).unwrap());
    }
    measurements.clear();
    socket.close();

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

// TODO: can we make this an embassy_executor::main?
#[embassy_executor::task]
async fn measure(measurements: &'static mut MeasurementBuffer) {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let analog = peripherals.APB_SARADC.split();
    let mut sensors = Sensors::new(
        analog.adc1,
        io.pins.gpio1,
        io.pins.gpio0,
        io.pins.gpio3,
        &mut system.peripheral_clock_control,
    );

    hal::embassy::init(&clocks, timer_group0.timer0);

    let mut led = Led::new(
        peripherals.DMA,
        peripherals.SPI2,
        io.pins.gpio7,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    led.color(1, 1, 1).await;

    let battery_millivolts = sensors.read_battery().await as u32;
    let temperature_millidegs = sensors.read_temperature().await;
    let rtc_ms = rtc.get_time_ms();
    measurements.push(Measurement {
        rtc_ms,
        temperature_millidegs,
        battery_millivolts,
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
    Timer::after(Duration::from_millis(100)).await;

    let mut delay = hal::Delay::new(&clocks);
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(sleep_ms));
    rtc.sleep_deep(&[&mut wake], &mut delay);
}

enum LedState {
    WaitingForNetwork,
    Idle,
    Sending,
}

#[embassy_executor::task]
async fn led_task(mut led: Led, state: Receiver<LedState>) {
    led.color(0, 0, 0).await;
    loop {
        let state = state.recv().await;

        match state {
            LedState::WaitingForNetwork => led.color(8, 0, 0).await,
            LedState::Idle => led.color(0, 0, 0).await,
            LedState::Sending => led.color(0, 0, 2).await,
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
