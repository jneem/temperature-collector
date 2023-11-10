use embassy_time::Timer;
use embedded_hal::{adc, digital};
use esp_hal_common::{
    adc::{AdcCalLine, AdcConfig, Attenuation},
    gpio::{Analog, GpioPin, Output, PushPull},
    peripheral::Peripheral,
};
use fixed::types::I20F12;
use hal::prelude::*;

use crate::Voltage;

async fn read_raw<ADC, Pin: adc::Channel<ADC>, Reader: adc::OneShot<ADC, u16, Pin>>(
    reader: &mut Reader,
    pin: &mut Pin,
) -> u16 {
    loop {
        let pin_value: nb::Result<u16, _> = reader.read(pin);
        if let Ok(val) = pin_value {
            return val;
        } else {
            // adc.read never returns an error other than WouldBlock, so we
            // can assume it's WouldBlock here.
            Timer::after_micros(50).await;
        }
    }
}

// Takes a trimmed mean of 10 measurements, throwing away the two extremes.
async fn read_trimmed<ADC, Pin: adc::Channel<ADC>, Reader: adc::OneShot<ADC, u16, Pin>>(
    reader: &mut Reader,
    pin: &mut Pin,
) -> Voltage {
    let mut total = read_raw(reader, pin).await;
    let mut max = total;
    let mut min = total;
    for _ in 0..9 {
        let reading = read_raw(reader, pin).await;
        max = max.max(reading);
        min = min.min(reading);
        total += reading;
    }

    total -= min;
    total -= max;

    let total = I20F12::from_num(total);
    let avg = total / 8;

    Voltage { mv: avg }
}

pub struct ActivatedSensor<ADC, SensorPin, ActivatePin> {
    adc: core::marker::PhantomData<ADC>,
    sensor_pin: SensorPin,
    activate_pin: ActivatePin,
}

impl<ADC, SensorPin, ActivatePin> ActivatedSensor<ADC, SensorPin, ActivatePin>
where
    SensorPin: adc::Channel<ADC>,
    ActivatePin: digital::v2::OutputPin,
{
    pub async fn read_voltage_averaged<Reader: adc::OneShot<ADC, u16, SensorPin>>(
        &mut self,
        reader: &mut Reader,
    ) -> Voltage {
        const REPS: i32 = 16;

        let mut ret = Voltage::ZERO;
        let _ = self.activate_pin.set_high();
        Timer::after_millis(5).await;
        for _ in 0..REPS {
            ret.mv += read_trimmed(reader, &mut self.sensor_pin).await.mv;
        }
        let _ = self.activate_pin.set_low();
        ret.mv /= REPS;
        ret
    }

    pub async fn read_voltage<Reader: adc::OneShot<ADC, u16, SensorPin>>(
        &mut self,
        reader: &mut Reader,
    ) -> Voltage {
        let _ = self.activate_pin.set_high();
        let ret = read_trimmed(reader, &mut self.sensor_pin).await;
        let _ = self.activate_pin.set_low();
        ret
    }
}

pub struct AdcBuilder<ADC> {
    config: AdcConfig<ADC>,
}

pub type AdcPin<ADC, const P: u8> = hal::adc::AdcPin<GpioPin<Analog, P>, ADC, AdcCalLine<ADC>>;

impl<ADC> Default for AdcBuilder<ADC>
where
    ADC: hal::adc::CalibrationAccess + hal::adc::AdcHasLineCal + hal::adc::AdcCalEfuse,
{
    fn default() -> Self {
        Self {
            config: Default::default(),
        }
    }
}

impl<ADC> AdcBuilder<ADC>
where
    ADC: hal::adc::CalibrationAccess + hal::adc::AdcHasLineCal + hal::adc::AdcCalEfuse,
{
    pub fn add_pin<const P: u8>(
        &mut self,
        pin: GpioPin<Analog, P>,
        atten: Attenuation,
    ) -> AdcPin<ADC, P>
    where
        GpioPin<Analog, P>: adc::Channel<ADC, ID = u8>,
    {
        self.config
            .enable_pin_with_cal::<_, AdcCalLine<ADC>>(pin, atten)
    }

    pub fn add_activated_pin<const SP: u8, const AP: u8>(
        &mut self,
        sensor_pin: GpioPin<Analog, SP>,
        activate_pin: GpioPin<Output<PushPull>, AP>,
        atten: Attenuation,
    ) -> ActivatedSensor<ADC, AdcPin<ADC, SP>, GpioPin<Output<PushPull>, AP>>
    where
        GpioPin<Analog, SP>: adc::Channel<ADC, ID = u8>,
    {
        let sensor_pin = self.add_pin(sensor_pin, atten);
        ActivatedSensor {
            adc: core::marker::PhantomData,
            sensor_pin,
            activate_pin,
        }
    }

    pub fn build(self, adc: impl Peripheral<P = ADC> + 'static) -> hal::adc::ADC<'static, ADC> {
        hal::adc::ADC::adc(adc, self.config).unwrap()
    }
}
