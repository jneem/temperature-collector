use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_println::println;
use hal::prelude::*;
use hal::{
    analog::adc,
    analog::adc::Attenuation,
    clock::{ClockControl, Clocks, CpuClock},
    gpio::IO,
    peripherals::ADC1,
    peripherals::{Peripherals, LPWR},
    rtc_cntl::sleep::TimerWakeupSource,
    rtc_cntl::Rtc,
    timer::TimerGroup,
};

use crate::{
    status_led::{LedHandle, OFF, WHITE},
    Measurement, Temperature,
};

pub struct Parts<BA, BR, TA, TR> {
    pub clocks: Clocks<'static>,
    pub rtc: LPWR,
    pub battery_activate: BA,
    pub battery_read: BR,
    pub temp_activate: TA,
    pub temp_read: TR,
    pub adc: ADC1,
}

pub async fn measure_new<const BUF_SIZE: usize, BA, BR, TA, TR>(
    measurements: &'static mut ArrayVec<Measurement, BUF_SIZE>,
    measurement_period: Duration,
    parts: Parts<BA, BR, TA, TR>,
    led: LedHandle,
) where
    BA: OutputPin,
    TA: OutputPin,
    BR: adc::AdcChannel,
    TR: adc::AdcChannel,
{
    let mut rtc = Rtc::new(parts.rtc, None);
    let mut builder = crate::analog::AdcBuilder::default();
    #[cfg(feature = "battery")]
    let mut battery_sensor = builder.add_activated_pin(
        parts.battery_read,
        parts.battery_activate,
        Attenuation::Attenuation2p5dB,
    );

    let mut temperature_sensor = builder.add_activated_pin(
        parts.temp_read,
        parts.temp_activate,
        Attenuation::Attenuation2p5dB,
    );
    let mut adc = builder.build(parts.adc);
    led.set(espilepsy::Cmd::Steady(WHITE)).await;

    #[cfg(feature = "battery")]
    let battery = Some(battery_sensor.read_voltage(&mut adc).await * 4);
    #[cfg(not(feature = "battery"))]
    let battery = None;

    let temperature =
        Temperature::from_tmp36_voltage(temperature_sensor.read_voltage_averaged(&mut adc).await);
    let rtc_ms = rtc.get_time_ms();
    measurements.push(Measurement {
        rtc_ms,
        battery,
        temperature,
        humidity: None,
    });

    led.set(espilepsy::Cmd::Steady(OFF)).await;
    Timer::after_millis(10).await;
    crate::sleep(measurements, measurement_period, &mut rtc, &parts.clocks)
}

pub async fn measure<const BUF_SIZE: usize>(
    spawner: Spawner,
    measurements: &'static mut ArrayVec<Measurement, BUF_SIZE>,
    measurement_period: Duration,
) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();

    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut builder = crate::analog::AdcBuilder::default();
    #[cfg(feature = "battery")]
    let mut battery_sensor = builder.add_activated_pin(
        io.pins.gpio3.into_analog(),
        io.pins.gpio2.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );

    // TODO: Some of the devices are already soldered with pin 0 here. This
    // makes it configurable, but it doesn't really scale. "if cfg!()" doesn't work
    // here because the two branches have to return the same type.
    #[cfg(feature = "activate-pin-0")]
    let activate_pin = io.pins.gpio0;
    #[cfg(not(feature = "activate-pin-0"))]
    let activate_pin = io.pins.gpio5;

    let mut temperature_sensor = builder.add_activated_pin(
        io.pins.gpio4.into_analog(),
        activate_pin.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );
    let mut adc = builder.build(peripherals.ADC1);

    hal::embassy::init(&clocks, timer_group0);

    let led = crate::status_led::init(&spawner, peripherals.RMT, io.pins.gpio7, &clocks);

    led.set(espilepsy::Cmd::Steady(WHITE)).await;

    #[cfg(feature = "battery")]
    let battery = Some(battery_sensor.read_voltage(&mut adc).await * 4);
    #[cfg(not(feature = "battery"))]
    let battery = None;

    let temperature =
        Temperature::from_tmp36_voltage(temperature_sensor.read_voltage_averaged(&mut adc).await);
    let rtc_ms = rtc.get_time_ms();
    measurements.push(Measurement {
        rtc_ms,
        battery,
        temperature,
        humidity: None,
    });

    led.set(espilepsy::Cmd::Steady(OFF)).await;
    let sleep_ms = if measurements.is_full() {
        0
    } else {
        let rtc_now = rtc.get_time_ms();
        let wakeup_time = rtc_now + measurement_period.as_millis();
        let quantized_wakeup_time = wakeup_time - wakeup_time % measurement_period.as_millis();
        quantized_wakeup_time.saturating_sub(rtc_now)
    };
    println!(
        "now is {}, planning to sleep for {} ms",
        rtc.get_time_ms(),
        sleep_ms
    );
    println!(
        "buf has {} measurements, last time is {}",
        measurements.len(),
        measurements.last().unwrap().rtc_ms
    );
    Timer::after(Duration::from_millis(100)).await;

    let mut delay = hal::delay::Delay::new(&clocks);
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(sleep_ms));
    rtc.sleep_deep(&[&mut wake], &mut delay);
}
