#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::_export::StaticCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::Configuration;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::EspWifiInitFor;
use hal::adc::{AdcCalLine, AdcConfig, AdcPin, Attenuation, ADC, ADC1};
use hal::gpio::{Analog, GpioPin};
use hal::systimer::SystemTimer;
use hal::{clock::ClockControl, peripherals::Peripherals, timer::TimerGroup, Rtc};
use hal::{prelude::*, Rng, IO};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

struct Sensor<const P: u8> {
    adc: ADC<'static, ADC1>,
    pin: AdcPin<GpioPin<Analog, P>, ADC1, AdcCalLine<ADC1>>,
    atten: Attenuation,
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

impl<const P: u8> Sensor<P>
where
    GpioPin<Analog, P>: embedded_hal::adc::Channel<ADC1, ID = u8>,
{
    fn read(&mut self) -> i16 {
        // TODO: remove the block
        let pin_value: u16 = nb::block!(self.adc.read(&mut self.pin)).unwrap();
        println!("read {}", pin_value);
        let mv = (pin_value as i32 * self.atten.ref_mv() as i32 / 4096) as i16;
        println!("mV {}", mv);
        let temp = mv - 500;
        println!("{}.{}°C", temp / 10, temp % 10);
        temp
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
    let pin =
        adc1_config.enable_pin_with_cal::<_, AdcCalLine<ADC1>>(io.pins.gpio3.into_analog(), atten);
    let adc1: ADC<'static, ADC1> = ADC::<ADC1>::adc(
        &mut system.peripheral_clock_control,
        analog.adc1,
        adc1_config,
    )
    .unwrap();
    let sensor = Sensor {
        adc: adc1,
        pin,
        atten,
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

    let executor = singleton!(Executor::new());
    executor.run(|spawner| {
        spawner
            .spawn(main_task(spawner, stack, controller, sensor))
            .unwrap();
    });

    unreachable!();
}

#[embassy_executor::task]
async fn main_task(
    spawner: Spawner,
    stack: &'static Stack<WifiDevice<'static>>,
    controller: WifiController<'static>,
    sensor: Sensor<3>,
) {
    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(stack)).unwrap();
    spawner.spawn(task(stack, sensor)).unwrap();
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
            Err(_e) => {
                println!("Failed to connect to wifi");
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
async fn task(stack: &'static Stack<WifiDevice<'static>>, mut sensor: Sensor<3>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

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

    let mut write = Writer::default();
    let mut req = Writer::default();
    Timer::after(Duration::from_millis(1_000)).await;

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let remote_endpoint = (Ipv4Address::new(192, 168, 86, 118), 3000);

    loop {
        println!("connecting...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }
        println!("connected!");

        write.clear();
        req.clear();
        let temp = sensor.read();
        write!(
            &mut write,
            "{{ \"sensor_id\": 1, \"temperature\": {} }}",
            temp
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
        Timer::after(Duration::from_millis(5000)).await;
    }
}
