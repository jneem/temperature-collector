use arrayvec::ArrayVec;
use esp_println::println;
use hal::prelude::*;
use static_cell::StaticCell;

use crate::Measurement;

type MeasurementBuffer = ArrayVec<Measurement, 64>;

// Workaround for https://github.com/esp-rs/esp-hal/issues/950
#[ram(rtc_fast, uninitialized)]
static mut _EMPTY: [u8; 8] = [0; 8];

#[ram(rtc_fast, uninitialized)]
static mut MEASUREMENT_BUFFER: MeasurementBuffer = ArrayVec::new_const();

// A static bool, to put a safe API around MEASUREMENT_BUFFER.
static INITED: StaticCell<()> = StaticCell::new();

pub fn get_measurement_buffer() -> &'static mut MeasurementBuffer {
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

    unsafe { core::mem::transmute(core::ptr::addr_of_mut!(MEASUREMENT_BUFFER)) }
}
