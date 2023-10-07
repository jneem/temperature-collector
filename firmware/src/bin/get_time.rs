#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use embassy_executor::_export::StaticCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::{Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiDevice, WifiMode};
use esp_wifi::EspWifiInitFor;
use hal::systimer::SystemTimer;
use hal::timer::TimerGroup;
use hal::{clock::ClockControl, peripherals::Peripherals};
use hal::{prelude::*, Rng, Rtc};
use temperature_firmware::{singleton, write_measurements, Measurement, Temperature, Voltage};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");

#[entry]
fn main() -> ! {
    let executor = singleton!(Executor::new(), Executor);

    executor.run(|spawner| {
        spawner.must_spawn(embassy_main(spawner));
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
async fn embassy_main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let rtc = Rtc::new(peripherals.RTC_CNTL);

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    hal::embassy::init(&clocks, timer_group0.timer0);

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
    let client_config = Configuration::Client(embedded_svc::wifi::ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    println!("Starting controller...");
    controller.start().await.unwrap();

    println!("Connecting...");
    controller.connect().await.unwrap();

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_secs(1)).await;
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let remote_endpoint = (Ipv4Address::new(192, 168, 86, 118), 8086);

    let date = {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        socket.connect(remote_endpoint).await.unwrap();

        let date = temperature_firmware::get_time_from_influx(&mut socket)
            .await
            .ok()
            .unwrap();
        socket.close();
        println!("got date {}", date);
        date
    };

    let measurement = Measurement {
        temperature: Temperature::from_degrees(77),
        battery: Some(Voltage::from_v(8)),
        rtc_ms: 0,
    };

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    socket.connect(remote_endpoint).await.unwrap();
    write_measurements(
        &mut socket,
        SENSOR_ID,
        date.assume_utc(),
        &rtc,
        core::iter::once(&measurement),
    )
    .await
    .ok()
    .unwrap();
    socket.close();
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
