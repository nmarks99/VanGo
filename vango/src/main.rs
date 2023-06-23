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
use esp_idf_svc::systime::EspSystemTime;

mod utils;
mod encoder;

use encoder::Encoder;

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

    let sys_timer = EspSystemTime {};

    // configure PWM pin on GPIO2
    let mut pwm_pin = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(50.Hz().into()),
        )?,
        peripherals.pins.gpio16
    )?;
    let max_duty = pwm_pin.get_max_duty();

    // GPIO14 sets motor direction
    let mut motor_dir = PinDriver::output(peripherals.pins.gpio14)?;
    motor_dir.set_low()?;

    // configure GPIO13 and GPIO12 to be inputs for encoder A and B
    let mut enc_a = peripherals.pins.gpio13;
    let mut enc_b = peripherals.pins.gpio12;
    let encoder = Encoder::new(peripherals.pcnt0, &mut enc_a, &mut enc_b)?;

    let mut enc_last = 0i64;
    
    pwm_pin.set_duty(max_duty)?;

    loop {
        
        let value = encoder.get_value()?;
        if value != enc_last {
            println!("value: {}, time = {}", value, sys_timer.now().as_millis());
            enc_last = value;
        }
        FreeRtos::delay_ms(100u32);
        
        // for duty in (0..max_duty).step_by(20) {
            // let enc_value = encoder.get_value()?;
            // if enc_value != enc_last {
                // println!("value: {enc_value}, duty: {duty} ");
                // enc_last = enc_value;
            // }
            // led.set_high()?;
            // pwm_pin.set_duty(duty)?;
            // FreeRtos::delay_ms(1000);
        // }
//
        // for duty in (0..max_duty+20).rev().step_by(20) {
            // let enc_value = encoder.get_value()?;
            // if enc_value != enc_last {
                // println!("value: {enc_value}, duty: {duty} ");
                // enc_last = enc_value;
            // }
            // led.set_low()?;
            // pwm_pin.set_duty(duty)?;
            // FreeRtos::delay_ms(1000);
        // }

        FreeRtos::delay_ms(1);
    }

}
