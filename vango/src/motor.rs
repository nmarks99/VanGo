#![allow(dead_code)]
#![allow(unused_imports)]
use anyhow;
use esp_idf_hal::gpio;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::ledc;
use esp_idf_hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::*;
use esp_idf_sys::EspError;

pub struct Mototr<'a> {
    ledc_driver: LedcDriver<'a>,
}

impl<'a> Motor<'a> {
    pub fn new(
        pin: impl Peripheral<P = impl OutputPin> + 'a,
        ledc_channel: impl Peripheral<P = impl LedcChannel> + 'a,
        ledc_timer: impl Peripheral<P = impl LedcTimer> + 'a,
    ) -> Result<Self, EspError> {
        let timer_driver = LedcTimerDriver::new(
            ledc_timer,
            &ledc::config::TimerConfig::new().frequency(Hertz(PWM_FREQ).into()),
        )?;
        let mut ledc_driver = LedcDriver::new(ledc_channel, timer_driver, pin)?;
        Ok(Self { ledc_driver })
    }
}
