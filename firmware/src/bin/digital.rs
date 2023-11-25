#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Duration;
use esp_backtrace as _;
use esp_println::println;
use hal::prelude::*;
use temperature_firmware::{ds18b20::measure, transmit::transmit};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");
const MS_PER_MEASUREMENT: u64 = 10_000;

#[main]
async fn main(spawner: Spawner) -> ! {
    let measurements = temperature_firmware::measurements::get_measurement_buffer();
    println!("buffer has {} measurements", measurements.len());

    if measurements.is_full() {
        transmit(
            spawner,
            measurements,
            SSID,
            PASSWORD,
            INFLUX_SERVER,
            SENSOR_ID,
            Duration::from_millis(MS_PER_MEASUREMENT),
        )
        .await;
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