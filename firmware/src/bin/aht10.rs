#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Duration;
use esp_backtrace as _;
use esp_println::println;
use hal::prelude::*;
use hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, timer::TimerGroup};
use temperature_firmware::{aht10::Parts, status_led};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");
const MS_PER_MEASUREMENT: u64 = 10_000;

#[main]
async fn main(spawner: Spawner) {
    let measurements = temperature_firmware::measurements::get_measurement_buffer();
    println!("buffer has {} measurements", measurements.len());

    if measurements.is_full() {
        temperature_firmware::transmit::transmit(
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
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        hal::embassy::init(&clocks, TimerGroup::new_async(peripherals.TIMG0, &clocks));

        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        let led = status_led::init(&spawner, peripherals.RMT, io.pins.gpio7, &clocks);
        temperature_firmware::aht10::measure(
            measurements,
            Duration::from_millis(MS_PER_MEASUREMENT),
            Parts {
                clocks,
                rtc: peripherals.LPWR,
                sda: io.pins.gpio8,
                scl: io.pins.gpio6,
                i2c: peripherals.I2C0,
            },
            led,
        )
        .await;
    }
}
