#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::_export::StaticCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_sync::channel;
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use embedded_svc::wifi::Configuration;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::EspWifiInitFor;
use hal::adc::{AdcCalLine, AdcConfig, AdcPin, Attenuation, ADC, ADC1};
use hal::clock::Clocks;
use hal::dma::DmaPriority;
use hal::gdma::{self, Gdma};
use hal::gpio::{Analog, GpioPin, Output, PushPull, Unknown};
use hal::spi::{FullDuplexMode, SpiMode};
use hal::system::PeripheralClockControl;
use hal::systimer::SystemTimer;
use hal::{clock::ClockControl, peripherals::Peripherals, timer::TimerGroup, Rtc};
use hal::{prelude::*, Rng, IO};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");

type Channel<T> =
    channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;
type Receiver<T> =
    channel::Receiver<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;
type Sender<T> =
    channel::Sender<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, 2>;

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
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
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
        let descriptors = singleton!([0u32; 8 * 3]);
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
    /// Reads the current temperature in milli-degrees
    async fn read_temperature(&mut self) -> i32 {
        let uv = read_uv(&mut self.adc, &mut self.temperature).await;
        println!("temp {} uV", uv);
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
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let analog = peripherals.APB_SARADC.split();
    let atten = Attenuation::Attenuation2p5dB;
    let mut adc1_config = AdcConfig::new();
    let temp_pin = SensorPin {
        pin: adc1_config
            .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(io.pins.gpio3.into_analog(), atten),
        atten,
    };
    let battery_pin = SensorPin {
        pin: adc1_config
            .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(io.pins.gpio1.into_analog(), atten),
        atten,
    };
    let battery_activate = io.pins.gpio0.into_push_pull_output();
    let adc1: ADC<'static, ADC1> = ADC::<ADC1>::adc(
        &mut system.peripheral_clock_control,
        analog.adc1,
        adc1_config,
    )
    .unwrap();
    let sensors = Sensors {
        temperature: temp_pin,
        battery: battery_pin,
        battery_activate,
        adc: adc1,
    };

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

    hal::embassy::init(&clocks, timer_group0.timer0);

    let dhcp_config = embassy_net::Config::dhcpv4(Default::default());
    let stack_resources = singleton!(StackResources::<3>::new());
    let stack = singleton!(Stack::new(
        wifi_interface,
        dhcp_config,
        stack_resources,
        1234,
    ));

    let led = Led::new(
        peripherals.DMA,
        peripherals.SPI2,
        io.pins.gpio7,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let led_channel = singleton!(Channel::new());

    let executor = singleton!(Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(main_task(
            spawner,
            stack,
            controller,
            sensors,
            led_channel.sender(),
        ));
        spawner.must_spawn(led_task(led, led_channel.receiver()));
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
async fn main_task(
    spawner: Spawner,
    stack: &'static Stack<WifiDevice<'static>>,
    controller: WifiController<'static>,
    sensors: Sensors,
    led_sender: Sender<LedState>,
) {
    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(stack)).unwrap();
    spawner.spawn(task(stack, sensors, led_sender)).unwrap();
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    use embedded_svc::wifi::Wifi;
    loop {
        if matches!(esp_wifi::wifi::get_wifi_state(), WifiState::StaConnected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(1000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(embedded_svc::wifi::ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting controller...");
            controller.start().await.unwrap();
        }
        println!("Connecting...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn task(
    stack: &'static Stack<WifiDevice<'static>>,
    mut sensors: Sensors,
    led: Sender<LedState>,
) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    let _ = led.try_send(LedState::WaitingForNetwork);
    loop {
        if stack.is_link_up() {
            break;
        }
        println!("not up yet...");
        Timer::after(Duration::from_millis(500)).await;
    }
    println!("up!");

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    let _ = led.try_send(LedState::Idle);

    let mut write = Writer::default();
    let mut req = Writer::default();
    Timer::after(Duration::from_millis(1_000)).await;

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let remote_endpoint = (Ipv4Address::new(192, 168, 86, 118), 3000);

    loop {
        let battery = sensors.read_battery().await;
        let temp = sensors.read_temperature().await;

        println!("connecting...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }
        println!("connected!");

        let _ = led.try_send(LedState::Sending);

        write.clear();
        req.clear();
        // FIXME: this is wrong for negative temperatures
        write!(
            &mut write,
            "{{ \"sensor_id\": {}, \"temperature\": {}.{:04}, \"battery\": {} }}",
            SENSOR_ID,
            temp / 1_000,
            temp % 1_000,
            battery
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
        socket.close();
        let _ = led.try_send(LedState::Idle);
        Timer::after(Duration::from_secs(5)).await;
    }
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
