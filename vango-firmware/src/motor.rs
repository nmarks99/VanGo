// use esp_idf_hal::gpio::{self, OutputPin};
// use esp_idf_hal::ledc;
// use esp_idf_hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver};
// use esp_idf_hal::peripheral::Peripheral;
// use esp_idf_hal::prelude::*;
// use esp_idf_sys::EspError;

// const MOTOR_PWM_FREQ: u32 = 80; // Hz

// pub struct Motor<'a> {
//     ledc_driver: LedcDriver<'a>,
//     direction_pin_driver: gpio::PinDriver<'a, gpio::AnyOutputPin, gpio::Output>,
// }

// impl<'a> Motor<'a> {
//     pub fn new(
//         pin: impl Peripheral<P = impl OutputPin> + 'a,
//         ledc_channel: impl Peripheral<P = impl LedcChannel> + 'a,
//         ledc_timer: impl Peripheral<P = impl LedcTimer> + 'a,
//     ) -> Result<Self, EspError> {
//         let timer_driver = LedcTimerDriver::new(
//             ledc_timer,
//             &ledc::config::TimerConfig::new().frequency(Hertz(MOTOR_PWM_FREQ).into()),
//         )?;
//         let ledc_driver = LedcDriver::new(ledc_channel, timer_driver, pin)?;
//         let mut direction_pin_driver = gpio::PinDriver::output(pin)?;
//         Ok(Self {
//             ledc_driver,
//             direction_pin_driver,
//         })
//     }

//     pub fn set_duty(&mut self, duty: u32) -> anyhow::Result<()> {
//         self.ledc_driver.set_duty(duty)?;
//         Ok(())
//     }
// }
