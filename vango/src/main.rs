use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::delay::FreeRtos;

use esp_idf_hal::gpio;
use esp_idf_hal::gpio::PinDriver;

use esp_idf_hal::adc;
use esp_idf_hal::adc::AdcChannelDriver;
use esp_idf_hal::adc::AdcDriver;

use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::config::TimerConfig;


// linear mapping between two ranges, similar to arduino's map function
fn map(x:u32, xmin:u32, xmax:u32, ymin:u32, ymax:u32) -> u32 {
    x*(ymax-ymin) / (xmax - xmin) + ymin
}


// these values were obtained experimentally
const POT_MIN: u32 = 128;
const POT_MAX: u32 = 3139;
const SERVO_MIN: u32 = 5;
const SERVO_MAX: u32 = 32;


fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    
    // this is the builtin LED
    let mut led = PinDriver::output(peripherals.pins.gpio25)?;

    // configure ADC2
    let mut adc = AdcDriver::new(
        peripherals.adc2,
        &adc::config::Config::new().calibration(true)
    )?;
    
    // setup GPIO4 as the ADC pin
    let mut adc_pin: AdcChannelDriver<'_, gpio::Gpio4, adc::Atten11dB<_>> =
        adc::AdcChannelDriver::new(peripherals.pins.gpio4)?;

    // configure PWM pin on GPIO2
    let mut channel = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(50.Hz().into()),
        )?,
        peripherals.pins.gpio16
    )?;

    
    // Control duty cycle (servo position) with adc value (potentiometer)
    loop {
        let adc_val: u16 = adc.read(&mut adc_pin).unwrap();
        channel.set_duty(map(adc_val.into(), POT_MIN, POT_MAX, SERVO_MIN, SERVO_MAX))?; 
        if adc_val == POT_MAX as u16 {
            led.set_high()?;
        }
        else {
            led.set_low()?;
        }
        FreeRtos::delay_ms(10);
    }

}
