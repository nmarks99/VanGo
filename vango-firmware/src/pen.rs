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

/// Pen object which provides methods for moving a servo
/// between "UP" and "DOWN" states
pub struct Pen<'a> {
    ledc_driver: LedcDriver<'a>,
    state: PenState
}

impl<'a> Pen<'a> {
    /// Returns a Pen configured on the specified pin
    ///
    /// # Arguments
    ///
    /// * `pin` - The PWM pin the servo is connected to
    /// * `ledc_channel` - The LedcChannel to use
    /// * `ledc_timer` - The LedcTimer to use
    ///
    /// # Examples
    ///
    /// ```
    /// use doc::Pen;
    /// use esp_idf_hal::peripherals::Peripherals;
    ///
    /// fn main() -> anyhow::Result() {
    /// let mut pen = Pen::new(
    ///    peripherals.pins.gpio15,
    ///    peripherals.ledc.channel2,
    ///    peripherals.ledc.timer2
    /// )?;
    /// pen.up()?;
    /// let state = pen.get_state()?
    /// }
    ///
    /// ```

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

    /// Moves the pen up
    pub fn up(&mut self) -> anyhow::Result<()> {
        self.ledc_driver.set_duty(UP_VALUE)?;
        self.state = PenState::UP;
        Ok(())
    }

    /// Moves the pen down
    pub fn down(&mut self) -> anyhow::Result<()> {
        self.ledc_driver.set_duty(DOWN_VALUE)?;
        self.state = PenState::DOWN;
        Ok(())
    }

    /// Returns the current state of the pen
    pub fn get_state(&self) -> anyhow::Result<PenState, EspError> {
        Ok(self.state)
    }

}

/// Converts a PenState into a boolean cooresponding
/// to whether or not the pen is in contact with the surface,
/// or in other words DOWN = true, UP = false
impl From<PenState> for bool {
    fn from(pen_state: PenState) -> Self {
        match pen_state {
            PenState::UP => false,
            PenState::DOWN => true
        }
    }
}
