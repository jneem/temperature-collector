use aht10_async::AHT10;
use arrayvec::ArrayVec;
use embassy_time::{Duration, Timer};
use esp_println::println;
use hal::{
    clock::Clocks,
    gpio::{InputPin, OutputPin},
    i2c::I2C,
    interrupt,
    peripheral::Peripheral,
    peripherals::{Interrupt, I2C0, LPWR},
    prelude::*,
    rtc_cntl::Rtc,
};

use crate::{
    sleep,
    status_led::{LedHandle, OFF, WHITE},
    Humidity, Measurement, Temperature,
};

pub struct Parts<SDA, SDC> {
    pub clocks: Clocks<'static>,
    pub rtc: LPWR,
    pub sda: SDA,
    pub scl: SDC,
    pub i2c: I2C0,
}

pub async fn measure<const BUF_SIZE: usize, SDA, SDC>(
    measurements: &'static mut ArrayVec<Measurement, BUF_SIZE>,
    measurement_period: Duration,
    parts: Parts<SDA, SDC>,
    led: LedHandle,
) where
    SDA: Peripheral<P = SDA> + InputPin + OutputPin,
    SDC: Peripheral<P = SDC> + InputPin + OutputPin,
{
    let mut rtc = Rtc::new(parts.rtc, None);
    let i2c = I2C::new_async(parts.i2c, parts.sda, parts.scl, 400u32.kHz(), &parts.clocks);
    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    led.set(espilepsy::Cmd::Steady(WHITE)).await;

    let delay = embassy_time::Delay;
    let mut device = AHT10::new(i2c, delay).await.unwrap();
    device.reset().await.unwrap();

    let (hum, temp) = device.read().await.unwrap();
    let temperature = Temperature::from_degrees(temp.celsius());
    let humidity = Humidity::from_relative(hum.rh());
    println!("read temperature {}", temperature.degrees());

    let rtc_ms = rtc.get_time_ms();
    let battery = None;
    measurements.push(Measurement {
        rtc_ms,
        battery,
        temperature,
        humidity: Some(humidity),
    });

    led.set(espilepsy::Cmd::Steady(OFF)).await;
    Timer::after_millis(10).await;
    sleep(measurements, measurement_period, &mut rtc, &parts.clocks)
}
