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

const UP_VALUE: u32 = 5;
const DOWN_VALUE: u32 = 32;
const PWM_FREQ: u32 = 50; // Hz

#[derive(Debug, Copy, Clone)]
pub enum PenState {
    UP,
    DOWN
}

pub struct Pen<'a> {
    ledc_driver: LedcDriver<'a>,
    state: PenState
}

impl<'a> Pen<'a> {
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
        ledc_driver.set_duty(UP_VALUE)?;
        Ok(Self { ledc_driver, state: PenState::UP })
    }

    pub fn up(&mut self) -> anyhow::Result<()> {
        self.ledc_driver.set_duty(UP_VALUE)?;
        self.state = PenState::UP;
        Ok(())
    }

    pub fn down(&mut self) -> anyhow::Result<()> {
        self.ledc_driver.set_duty(DOWN_VALUE)?;
        self.state = PenState::DOWN;
        Ok(())
    }

    pub fn get_state(&self) -> anyhow::Result<PenState, EspError> {
        Ok(self.state)
    }

}
