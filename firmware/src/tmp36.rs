use arrayvec::ArrayVec;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal_common::{
    adc::Attenuation,
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    rtc_cntl::sleep::TimerWakeupSource,
    timer::TimerGroup,
    Rtc, IO,
};
use esp_println::println;
use hal::prelude::*;

use crate::{
    status_led::{OFF, WHITE},
    Measurement, Temperature,
};

pub async fn measure<const BUF_SIZE: usize>(
    spawner: Spawner,
    measurements: &'static mut ArrayVec<Measurement, BUF_SIZE>,
    measurement_period: Duration,
) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let analog = peripherals.APB_SARADC.split();

    let mut builder = crate::analog::AdcBuilder::default();
    #[cfg(feature = "battery")]
    let mut battery_sensor = builder.add_activated_pin(
        io.pins.gpio3.into_analog(),
        io.pins.gpio2.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );
    let mut temperature_sensor = builder.add_activated_pin(
        io.pins.gpio4.into_analog(),
        io.pins.gpio0.into_push_pull_output(),
        Attenuation::Attenuation2p5dB,
    );
    let mut adc = builder.build(analog.adc1);

    hal::embassy::init(&clocks, timer_group0.timer0);

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

    let mut delay = hal::Delay::new(&clocks);
    let mut wake = TimerWakeupSource::new(core::time::Duration::from_millis(sleep_ms));
    rtc.sleep_deep(&[&mut wake], &mut delay);
}
