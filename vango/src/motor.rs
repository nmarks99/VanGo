use anyhow;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::EspError;

use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::{LedcChannel, LedcTimer};
use esp_idf_hal::prelude::*;

pub struct Motor<'a> {
    pwm_driver: LedcDriver<'a>,
}

impl<'a> Motor<'a> {
    pub fn new(
        pwm_pin: impl Peripheral<P = impl OutputPin> + 'a,
        direction_pin: impl Peripheral<P = impl OutputPin> + 'a,
        ledc_channel: impl Peripheral<P = impl LedcChannel> + 'a,
        ledc_timer: impl Peripheral<P = impl LedcTimer> + 'a,
    ) -> anyhow::Result<Self, EspError> {
        // setup LEDC (PWM) driver
        let pwm_driver = LedcDriver::new(
            ledc_channel,
            LedcTimerDriver::new(ledc_timer, &TimerConfig::new().frequency(80.Hz().into()))?,
            pwm_pin,
        )?;

        // let mut direction_driver = PinDriver::output(direction_pin)?;

        Ok(Self {
            pwm_driver,
        })
    }

    pub fn set_duty(&mut self, value: u32) -> anyhow::Result<()> {
        self.pwm_driver.set_duty(value)?;
        Ok(())
    }

    // pub fn set_direction(self, direction: bool) -> anyhow::Result<()> {
    //     if direction {
    //         self.pwm_driver.se
    //     }
    // }
}
