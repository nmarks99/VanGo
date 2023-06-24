use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::delay::FreeRtos;

use esp_idf_hal::gpio;
use esp_idf_hal::gpio::PinDriver;

// use esp_idf_hal::adc;
// use esp_idf_hal::adc::AdcChannelDriver;
// use esp_idf_hal::adc::AdcDriver;

use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_svc::systime::EspSystemTime;
// use esp_idf_svc::notify::EspNotify;
// use critical_section::{Mutex, CriticalSection};
use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};

mod utils;
// mod encoder;
// use encoder::Encoder;

const GEAR_RATIO: u32 = 150;
const ENCODER_MULT: u32 = 14;


static LAST_TIME_ATOMIC: AtomicU32 = AtomicU32::new(0);
static ELAPSED_TIME_ATOMIC: AtomicU32 = AtomicU32::new(0);
static ENC_INTERRUPT_FLAG: AtomicBool = AtomicBool::new(false);

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();

    // builtin LED
    // let mut led = PinDriver::output(peripherals.pins.gpio25)?;

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

    // set up interrupt on enc A
    let mut enc_a_driver = PinDriver::input(peripherals.pins.gpio13)?;
    // let pin_number = enc_a_driver.pin();
    enc_a_driver.set_pull(gpio::Pull::Up)?;
    enc_a_driver.set_interrupt_type(gpio::InterruptType::AnyEdge)?;

    // ISR
    unsafe {
        enc_a_driver.subscribe(move || {
            ENC_INTERRUPT_FLAG.store(true, Ordering::SeqCst);
            let current_time = sys_timer.now().as_micros() as u32;
            let last_time = LAST_TIME_ATOMIC.load(Ordering::SeqCst);
            if last_time < current_time {
                ELAPSED_TIME_ATOMIC.store(current_time-last_time, Ordering::SeqCst);
            }
            LAST_TIME_ATOMIC.store(current_time, Ordering::SeqCst);
        })?;
    }

    // set motor speed
    pwm_pin.set_duty(max_duty)?;

    loop {
        FreeRtos::delay_ms(100);
        let rpm = get_motor_rpm();
        println!("speed = {} rpm", rpm);
    }

}

fn get_motor_rpm() -> f32 {
    let mut rpm: f32; 
    if ENC_INTERRUPT_FLAG.load(Ordering::SeqCst) {
        let elapsed = ELAPSED_TIME_ATOMIC.load(Ordering::SeqCst);
        println!("{}",elapsed);
        if elapsed == 0 {
            rpm = 0.0
        }
        else {
            rpm = 1.0 / (elapsed as f32);
            rpm = rpm * 1000000.0 * 60.0 / GEAR_RATIO as f32 / ENCODER_MULT as f32;
        }
        ENC_INTERRUPT_FLAG.store(false,Ordering::SeqCst);
    }
    else {
        rpm = 0.0;
    }

    rpm
}





