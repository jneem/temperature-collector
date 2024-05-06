use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_println::println;
use esp_wifi::{
    wifi::{Configuration, WifiController, WifiDevice, WifiError, WifiStaDevice},
    EspWifiInitFor,
};
use hal::prelude::*;
use hal::{
    clock::ClockControl, gpio::IO, peripherals::Peripherals, rng::Rng,
    rtc_cntl::sleep::TimerWakeupSource, rtc_cntl::Rtc, systimer::SystemTimer, timer::TimerGroup,
};

use crate::{
    get_time_from_influx, singleton,
    status_led::{LedHandle, BLUE, GREEN, OFF, RED},
    write_measurements, GetTimeError, Measurement,
};

async fn connect(
    mut controller: WifiController<'static>,
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    led: &LedHandle,
    ssid: &str,
    password: &str,
) -> Result<(), WifiError> {
    let client_config = Configuration::Client(esp_wifi::wifi::ClientConfiguration {
        ssid: ssid.try_into().unwrap(),
        password: password.try_into().unwrap(),
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

pub enum SendMeasurementError {
    Connect(embassy_net::tcp::ConnectError),
    Get(GetTimeError),
    Send(crate::SendMeasurementError),
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

impl From<crate::SendMeasurementError> for SendMeasurementError {
    fn from(v: crate::SendMeasurementError) -> Self {
        Self::Send(v)
    }
}

impl From<GetTimeError> for SendMeasurementError {
    fn from(v: GetTimeError) -> Self {
        Self::Get(v)
    }
}

async fn send_measurements(
    influx_server: &str,
    sensor_id: &str,
    stack: &Stack<WifiDevice<'static, WifiStaDevice>>,
    rtc: &Rtc<'_>,
    measurements: &[Measurement],
) -> Result<(), SendMeasurementError> {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    let address = stack.dns_query(influx_server, DnsQueryType::A).await?;
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
    write_measurements(
        &mut socket,
        sensor_id,
        date.assume_utc(),
        rtc,
        measurements.iter(),
    )
    .await?;

    Ok(())
}

pub async fn transmit<const CAP: usize>(
    spawner: Spawner,
    measurements: &'static mut ArrayVec<Measurement, CAP>,
    ssid: &str,
    password: &str,
    influx_server: &str,
    sensor_id: &str,
    measurement_period: Duration,
) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut rtc = Rtc::new(peripherals.LPWR, None);

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    hal::embassy::init(&clocks, timer_group0);

    let led = crate::status_led::init(&spawner, peripherals.RMT, io.pins.gpio7, &clocks);

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

    spawner.must_spawn(crate::net_task(stack));

    if connect(controller, stack, &led, ssid, password)
        .await
        .is_err()
    {
        led.set(espilepsy::Cmd::Steady(OFF)).await;
        Timer::after(Duration::from_millis(100)).await;
        let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(100));
        let mut delay = hal::delay::Delay::new(&clocks);
        rtc.sleep_deep(&[&mut wake], &mut delay);
    }

    led.set(espilepsy::Cmd::Blinky {
        color0: GREEN,
        color1: OFF,
        period: Duration::from_millis(500),
    })
    .await;

    let result = send_measurements(influx_server, sensor_id, stack, &rtc, &*measurements).await;
    if result.is_ok() {
        measurements.clear();
    }

    led.set(espilepsy::Cmd::Steady(OFF)).await;
    Timer::after_millis(100).await;
    let rtc_now = rtc.get_time_ms();
    let wakeup_time = rtc_now + measurement_period.as_millis();
    let quantized_wakeup_time = wakeup_time - wakeup_time % measurement_period.as_millis();
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(
        quantized_wakeup_time.saturating_sub(rtc_now),
    ));
    let mut delay = hal::delay::Delay::new(&clocks);
    rtc.sleep_deep(&[&mut wake], &mut delay);
}
