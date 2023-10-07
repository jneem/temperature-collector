#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use core::fmt::{Display, Write};

use arrayvec::ArrayVec;
use embassy_executor::_export::StaticCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::{ConnectError, TcpSocket};
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_sync::channel;
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use embedded_svc::wifi::{Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiError, WifiMode};
use esp_wifi::EspWifiInitFor;
use fixed::types::I20F12;
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

use temperature_firmware::{singleton, Measurement, Temperature, Voltage};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_TOKEN: &str = env!("INFLUX_TOKEN");
const MS_PER_MEASUREMENT: u64 = 10_000;

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

struct SensorPin<const P: u8> {
    atten: Attenuation,
    pin: AdcPin<GpioPin<Analog, P>, ADC1, AdcCalLine<ADC1>>,
}

struct Writer {
    buf: [u8; 1024],
    cursor: usize,
}

impl Default for Writer {
    fn default() -> Self {
        Self {
            buf: [0; 1024],
            cursor: 0,
        }
    }
}

impl Writer {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.cursor]).unwrap()
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.cursor]
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

macro_rules! write_buf {
    ($w:expr, $buf:expr, $($rest:tt)*) => {
        {
            $buf.clear();
            write!(&mut $buf, $($rest)*).unwrap();
            embedded_io::asynch::Write::write_all(&mut $w, $buf.as_bytes()).await
        }
    }
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

async fn read_voltage<const P: u8>(adc: &mut ADC<'_, ADC1>, pin: &mut SensorPin<P>) -> Voltage
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

    // Each of the 8 readings is at most 2^12, so the total is at most 2^15.
    // ref_mv is around 2^10, so the product easily fits in 32 bits.
    let avg = (total as i32 * pin.atten.ref_mv() as i32) / 8;

    // The units of ref_mv are such that we need to divide by 2^12 to get back
    // to mV. Casting to I20F12 serves the same purpose, while keeping the precision.
    Voltage {
        mv: I20F12::from_bits(avg),
    }
}

struct Sensors {
    adc: ADC<'static, ADC1>,
    battery: SensorPin<3>,
    battery_activate: GpioPin<Output<PushPull>, 2>,
    temperature: SensorPin<4>,
    temperature_activate: GpioPin<Output<PushPull>, 0>,
}

impl Sensors {
    fn new(
        adc: ADC1,
        battery: GpioPin<Unknown, 3>,
        battery_activate: GpioPin<Unknown, 2>,
        temperature: GpioPin<Unknown, 4>,
        temperature_activate: GpioPin<Unknown, 0>,
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
        let adc1: ADC<'static, ADC1> = ADC::<ADC1>::adc(pcc, adc, adc_config).unwrap();
        Sensors {
            temperature: temp_pin,
            temperature_activate: temperature_activate.into_push_pull_output(),
            battery: battery_pin,
            battery_activate: battery_activate.into_push_pull_output(),
            adc: adc1,
        }
    }

    async fn read_temperature(&mut self) -> Temperature {
        let mut ret = Temperature::default();
        const REPS: i32 = 16;

        let _ = self.temperature_activate.set_high();
        for _ in 0..REPS {
            let v = read_voltage(&mut self.adc, &mut self.temperature).await;
            ret.deg += Temperature::from_tmp36_voltage(v).deg;
            println!("temp {}", v);
        }
        let _ = self.temperature_activate.set_low();
        ret.deg /= REPS;
        ret
    }

    /// Returns the current battery voltage, in millivolts.
    async fn read_battery(&mut self) -> Voltage {
        let _ = self.battery_activate.set_high();
        Timer::after(Duration::from_millis(5)).await;
        let mut v = read_voltage(&mut self.adc, &mut self.battery).await;
        let _ = self.battery_activate.set_low();
        println!("batt {}", v);

        // The voltage measurement is taken between a 68k resistor and a 10k
        // resistor, so our measurement is 10/78ths of the original voltage.
        v.mv *= 78;
        v.mv /= 10;
        v
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

async fn connect(
    mut controller: WifiController<'static>,
    stack: &'static Stack<WifiDevice<'static>>,
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
    Connect(ConnectError),
    Tcp(embassy_net::tcp::Error),
}

impl From<embassy_net::tcp::Error> for SendMeasurementError {
    fn from(v: embassy_net::tcp::Error) -> Self {
        Self::Tcp(v)
    }
}

impl From<ConnectError> for SendMeasurementError {
    fn from(e: ConnectError) -> Self {
        Self::Connect(e)
    }
}

async fn send_measurements(
    stack: &Stack<WifiDevice<'_>>,
    rtc: &Rtc<'_>,
    measurements: &MeasurementBuffer,
) -> Result<(), SendMeasurementError> {
    let mut write = Writer::default();
    let mut req = Writer::default();
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // FIXME: look up a domain name
    let remote_endpoint = (Ipv4Address::new(192, 168, 86, 118), 8086);

    // TODO: This is super bizarre. On deep sleep, the rtc_ms field in the
    // first measurement always gets zeroed. We fix it by always ignoring the
    // first measurement. (Below this loop, we stick in a dummy first measurement
    // so that we aren't actually skipping any data.)
    for meas in measurements.iter().skip(1) {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        socket.connect(remote_endpoint).await?;

        write.clear();
        req.clear();

        let now = rtc.get_time_ms();
        let ms_ago = now.saturating_sub(meas.rtc_ms);

        #[cfg(feature = "battery")]
        write!(
            &mut write,
            "temperature,sensor_id={} temperature={},battery={} {}\n",
            SENSOR_ID,
            meas.temperature.degrees(),
            // The collector expects an integer, so round it.
            meas.battery.unwrap().mv().round(),
            // FIXME: influx doesn't natively support relative times, see the workaround in
            // https://github.com/influxdata/influxdb/issues/16166
            ms_ago
        )
        .unwrap();

        #[cfg(not(feature = "battery"))]
        write!(
            &mut write,
            "{{ \"sensor_id\": {}, \"temperature\": {}, \"ms_ago\": {} }}",
            SENSOR_ID, meas.temperature.deg, ms_ago
        )
        .unwrap();

        use embedded_io::asynch::Write;
        write_buf!(socket, req, "POST /collect HTTP/1.0\r\n").unwrap();
        write_buf!(socket, req, "Connection: Close\r\n").unwrap();
        write_buf!(socket, req, "Authorization: Token {}\r\n", INFLUX_TOKEN).unwrap();
        write_buf!(socket, req, "Content-type: text/plain; charset=utf-8\r\n").unwrap();
        write_buf!(socket, req, "Content-length: {}\r\n", write.len()).unwrap();
        write_buf!(socket, req, "\r\n").unwrap();
        write!(&mut req, "{}", write.as_str()).unwrap();

        println!("sending request {}", req.as_str());

        socket.write_all(req.as_str().as_bytes()).await?;

        let mut read_buf = [0; 256];
        while socket.read(&mut read_buf).await? > 0 {}
        socket.close();
    }

    Ok(())
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
    let (wifi_interface, controller) =
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
        io.pins.gpio3,
        io.pins.gpio2,
        io.pins.gpio4,
        io.pins.gpio0,
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

    #[cfg(feature = "battery")]
    let battery = Some(sensors.read_battery().await);
    #[cfg(not(feature = "battery"))]
    let battery = None;

    let temperature = sensors.read_temperature().await;
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
        let state = state.recv().await;

        match state {
            LedState::WaitingForNetwork => led.color(8, 0, 0).await,
            LedState::Idle => led.color(0, 0, 0).await,
            LedState::Sending => led.color(0, 0, 2).await,
            LedState::Other => led.color(0, 1, 2).await,
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
