use embassy_time::Timer;
use embedded_hal::digital::{self, OutputPin};
use fixed::types::I20F12;
use hal::analog::adc::{
    self, AdcCalLine, AdcCalScheme, AdcChannel, AdcConfig, Attenuation, CalibrationAccess,
    RegisterAccess,
};
use hal::peripheral::Peripheral;
use hal::prelude::*;

use crate::Voltage;

pub trait AdcReader<Pin> {
    type Error;

    fn read(&mut self, pin: &mut Pin) -> nb::Result<u16, Self::Error>;
}

impl<'d, ADCI, Pin> AdcReader<AdcPin<Pin, ADCI>> for hal::analog::adc::ADC<'d, ADCI>
where
    ADCI: RegisterAccess + CalibrationAccess + 'd,
    AdcCalLine<ADCI>: AdcCalScheme<ADCI>,
    Pin: AdcChannel,
{
    type Error = ();

    fn read(&mut self, pin: &mut AdcPin<Pin, ADCI>) -> nb::Result<u16, ()> {
        self.read_oneshot(pin)
    }
}

async fn read_raw<Pin, Reader: AdcReader<Pin>>(reader: &mut Reader, pin: &mut Pin) -> u16 {
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
async fn read_trimmed<Pin, Reader: AdcReader<Pin>>(reader: &mut Reader, pin: &mut Pin) -> Voltage {
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

pub struct ActivatedSensor<SensorPin, ActivatePin> {
    sensor_pin: SensorPin,
    activate_pin: ActivatePin,
}

impl<SensorPin, ActivatePin> ActivatedSensor<SensorPin, ActivatePin>
where
    ActivatePin: digital::OutputPin,
{
    pub async fn read_voltage_averaged<Reader: AdcReader<SensorPin>>(
        &mut self,
        reader: &mut Reader,
    ) -> Voltage {
        const REPS: i32 = 8;

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

    pub async fn read_voltage<Reader: AdcReader<SensorPin>>(
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

pub type AdcPin<P, ADC> = adc::AdcPin<P, ADC, AdcCalLine<ADC>>;

impl<ADC> Default for AdcBuilder<ADC>
where
    ADC: adc::CalibrationAccess + adc::AdcHasLineCal,
{
    fn default() -> Self {
        Self {
            config: Default::default(),
        }
    }
}

impl<ADC> AdcBuilder<ADC>
where
    ADC: adc::CalibrationAccess + adc::AdcHasLineCal,
    AdcCalLine<ADC>: AdcCalScheme<ADC>,
{
    pub fn add_pin<P>(&mut self, pin: P, atten: Attenuation) -> AdcPin<P, ADC>
    where
        P: AdcChannel,
    {
        self.config
            .enable_pin_with_cal::<_, AdcCalLine<ADC>>(pin, atten)
    }

    pub fn add_activated_pin<SP, AP>(
        &mut self,
        sensor_pin: SP,
        activate_pin: AP,
        atten: Attenuation,
    ) -> ActivatedSensor<AdcPin<SP, ADC>, AP>
    where
        SP: AdcChannel,
        AP: OutputPin,
    {
        let sensor_pin = self.add_pin(sensor_pin, atten);
        ActivatedSensor {
            sensor_pin,
            activate_pin,
        }
    }

    pub fn build(self, adc: impl Peripheral<P = ADC> + 'static) -> adc::ADC<'static, ADC> {
        adc::ADC::new(adc, self.config)
    }
}
