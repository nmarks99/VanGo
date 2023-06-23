use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::delay::FreeRtos;

// use esp_idf_hal::gpio;
use esp_idf_hal::gpio::PinDriver;

// use esp_idf_hal::adc;
// use esp_idf_hal::adc::AdcChannelDriver;
// use esp_idf_hal::adc::AdcDriver;

use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::config::TimerConfig;

mod utils;


// these values were obtained experimentally
// const POT_MIN: u32 = 128;
// const POT_MAX: u32 = 3139;
// const SERVO_MIN: u32 = 5;
// const SERVO_MAX: u32 = 32;


fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    
    // this is the builtin LED
    let mut led = PinDriver::output(peripherals.pins.gpio25)?;

    // configure PWM pin on GPIO2
    let mut pwm_pin = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(50.Hz().into()),
        )?,
        peripherals.pins.gpio16
    )?;

    // configure GPIO13 and GPIO12 to be inputs for encoder A and B
    // let enc_a = PinDriver::input(peripherals.pins.gpio13)?;
    // let enc_a = PinDriver::input(peripherals.pins.gpio12)?;

    // GPIO14 sets motor direction
    let mut motor_dir = PinDriver::output(peripherals.pins.gpio14)?;
    motor_dir.set_low()?;
    
    let max_duty = pwm_pin.get_max_duty();

    loop {

        for duty in (0..max_duty).step_by(20) {
            led.set_high()?;
            pwm_pin.set_duty(duty)?;
            println!("Duty cycle = {duty}");
            FreeRtos::delay_ms(1000);
        }

        for duty in (0..max_duty+20).rev().step_by(20) {
            led.set_low()?;
            pwm_pin.set_duty(duty)?;
            println!("Duty cycle = {duty}");
            FreeRtos::delay_ms(1000);
        }

        FreeRtos::delay_ms(1);
    }

}
