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
use critical_section::Mutex;

mod utils;
mod encoder;
use encoder::Encoder;

const GEAR_RATIO: u32 = 150;
const ENCODER_MULT: u32 = 14;



// use esp_idf_sys::{
//     esp, gpio_config, gpio_config_t, gpio_install_isr_service, gpio_int_type_t_GPIO_INTR_POSEDGE,
//     gpio_isr_handler_add, gpio_mode_t_GPIO_MODE_INPUT, xQueueGenericCreate, xQueueGiveFromISR,
//     xQueueReceive, QueueHandle_t, ESP_INTR_FLAG_IRAM,
// };
// use std::ptr;

// // This `static mut` holds the queue handle we are going to get from `xQueueGenericCreate`.
// // This is unsafe, but we are careful not to enable our GPIO interrupt handler until after this value has been initialised, and then never modify it again
// static mut EVENT_QUEUE: Option<QueueHandle_t> = None;

// #[link_section = ".iram0.text"]
// unsafe extern "C" fn my_interrupt(_: *mut core::ffi::c_void) {
//     xQueueGiveFromISR(EVENT_QUEUE.unwrap(), std::ptr::null_mut());
// }



fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();

    // builtin LED
    let mut led = PinDriver::output(peripherals.pins.gpio25)?;

    // system timer to get uptime
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

    // set max speed
    pwm_pin.set_duty(max_duty/2)?;

  // uint32_t currA = micros();
  // if (lastA < currA) {
    // // did not wrap around
    // float rev = currA - lastA;  // us
    // rev = 1.0 / rev;            // rev per us
    // rev *= 1000000;             // rev per sec
    // rev *= 60;                  // rev per min
    // rev /= GEARING;             // account for gear ratio
    // rev /= ENCODERMULT;         // account for multiple ticks per rotation
    // RPM = rev;
  // }
  // lastA = currA;

    loop {

        let enc_count = encoder.get_value()?;
        if enc_count != enc_last {
            enc_last = enc_count;
            println!("Count = {}", enc_count);
        }

        FreeRtos::delay_ms(100);

    }

}
