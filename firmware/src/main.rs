#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::{Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{
    wifi::{WifiController, WifiDevice, WifiError, WifiStaDevice},
    EspWifiInitFor,
};
use fixed::types::I20F12;
use hal::{
    clock::ClockControl, peripherals::Peripherals, prelude::*, rtc_cntl::sleep::TimerWakeupSource,
    systimer::SystemTimer, timer::TimerGroup, Rng, Rtc, IO,
};
use static_cell::StaticCell;

use temperature_firmware::{
    get_time_from_influx, singleton,
    status_led::{LedHandle, BLUE, GREEN, OFF, RED},
    tmp36::measure,
    write_measurements, GetTimeError, Measurement, Temperature,
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");
const MS_PER_MEASUREMENT: u64 = 10_000;

type MeasurementBuffer = ArrayVec<Measurement, 64>;

// Workaround for https://github.com/esp-rs/esp-hal/issues/950
#[ram(rtc_fast, uninitialized)]
static mut _EMPTY: [u8; 8] = [0; 8];

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

#[main]
async fn main(spawner: Spawner) -> ! {
    let measurements = get_measurement_buffer();
    println!("buffer has {} measurements", measurements.len());

    if measurements.is_full() {
        transmit(spawner, measurements).await;
    } else {
        measure(
            spawner,
            measurements,
            Duration::from_millis(MS_PER_MEASUREMENT),
        )
        .await;
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
    led: &LedHandle,
) -> Result<(), WifiError> {
    let client_config = Configuration::Client(embedded_svc::wifi::ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    controller.set_configuration(&client_config)?;
    println!("Starting controller...");
    led.set(espilepsy::Cmd::Steady(RED)).await;
    // TODO: add a timeout
    controller.start().await?;

    led.set(espilepsy::Cmd::Blinky {
        color0: RED,
        color1: BLUE,
        period: Duration::from_millis(500),
    })
    .await;

    println!("Connecting...");
    // TODO: add a timeout
    controller.connect().await?;

    led.set(espilepsy::Cmd::Blinky {
        color0: BLUE,
        color1: GREEN,
        period: Duration::from_millis(500),
    })
    .await;
    println!("Waiting to get IP address...");
    for _ in 0..40 {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            led.set(espilepsy::Cmd::Steady(GREEN)).await;
            return Ok(());
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    led.set(espilepsy::Cmd::Steady(OFF)).await;

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

    let led =
        temperature_firmware::status_led::init(&spawner, peripherals.RMT, io.pins.gpio7, &clocks);

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

    if connect(controller, stack, &led).await.is_err() {
        led.set(espilepsy::Cmd::Steady(OFF)).await;
        Timer::after(Duration::from_millis(100)).await;
        let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(100));
        let mut delay = hal::Delay::new(&clocks);
        rtc.sleep_deep(&[&mut wake], &mut delay);
    }

    led.set(espilepsy::Cmd::Blinky {
        color0: GREEN,
        color1: OFF,
        period: Duration::from_millis(500),
    })
    .await;

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

    led.set(espilepsy::Cmd::Steady(OFF)).await;
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
