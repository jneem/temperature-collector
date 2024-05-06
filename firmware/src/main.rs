#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_time::Duration;
use esp_backtrace as _;
use esp_println::println;
use hal::prelude::*;
use static_cell::StaticCell;
use temperature_firmware::{tmp36::measure, transmit::transmit, Measurement};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SENSOR_ID: &str = env!("SENSOR_ID");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");
const MS_PER_MEASUREMENT: u64 = 15_000;

type MeasurementBuffer = ArrayVec<Measurement, 256>;

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

    unsafe { &mut *core::ptr::addr_of_mut!(MEASUREMENT_BUFFER) }
}

#[main]
async fn main(spawner: Spawner) {
    let measurements = get_measurement_buffer();
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
}
