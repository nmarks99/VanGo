// basics
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;

// GPIO
use esp_idf_hal::gpio;
use esp_idf_hal::gpio::PinDriver;

// ADC
use esp_idf_hal::adc;
use esp_idf_hal::adc::AdcChannelDriver;
use esp_idf_hal::adc::AdcDriver;
use esp_idf_hal::adc::Atten11dB;

// LEDC (PWM)
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::LedcTimerDriver;

// TIME
use esp_idf_svc::systime::EspSystemTime;

// user modules
mod neopixel;
use neopixel::Rgb;
use neopixel::Neopixel;
mod utils;

const GEAR_RATIO: u32 = 150; // TODO: check this
const ENCODER_MULT: u32 = 14;

// these values were obtained experimentally
const POT_MIN: u32 = 128;
const POT_MAX: u32 = 3139;

static LAST_TIME_ATOMIC: AtomicU32 = AtomicU32::new(0);
static ELAPSED_TIME_ATOMIC: AtomicU32 = AtomicU32::new(0);
static ENC_INTERRUPT_FLAG: AtomicBool = AtomicBool::new(false);

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();

    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio12, peripherals.rmt.channel0)?;
    neo.set_color("cyan", 0.2)?;

    // system timer to get uptime
    let sys_timer = EspSystemTime {};

    // configure PWM pin on GPIO2
    let mut pwm_pin1 = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(50.Hz().into()),
        )?,
        peripherals.pins.gpio15,
    )?;

    // GPIO14 sets motor direction
    let mut motor_dir = PinDriver::output(peripherals.pins.gpio32)?;
    motor_dir.set_low()?;

    let max_duty = pwm_pin1.get_max_duty();

    // Setup adc driver
    let mut adc_driver = AdcDriver::new(
        peripherals.adc2,
        &adc::config::Config::new().calibration(true),
    )?;

    // setup analog input for pot1 and pot2
    let mut pot1: AdcChannelDriver<'_, gpio::Gpio26, Atten11dB<_>> =
        adc::AdcChannelDriver::new(peripherals.pins.gpio26)?;
    let mut pot2: AdcChannelDriver<'_, gpio::Gpio25, Atten11dB<_>> =
        adc::AdcChannelDriver::new(peripherals.pins.gpio25)?;

    // set up interrupt on enc A
    let mut enc_a_driver = PinDriver::input(peripherals.pins.gpio27)?;
    enc_a_driver.set_pull(gpio::Pull::Up)?;
    enc_a_driver.set_interrupt_type(gpio::InterruptType::AnyEdge)?;

    // ISR
    unsafe {
        enc_a_driver.subscribe(move || {
            ENC_INTERRUPT_FLAG.store(true, Ordering::SeqCst);
            let current_time = sys_timer.now().as_micros() as u32;
            let last_time = LAST_TIME_ATOMIC.load(Ordering::SeqCst);
            if last_time < current_time {
                ELAPSED_TIME_ATOMIC.store(current_time - last_time, Ordering::SeqCst);
            }
            LAST_TIME_ATOMIC.store(current_time, Ordering::SeqCst);
        })?;
    }

    // set motor speed
    pwm_pin1.set_duty(max_duty)?;

    loop {
        FreeRtos::delay_ms(50);
        let pot1_val = adc_driver.read(&mut pot1).unwrap();
        let pot2_val = adc_driver.read(&mut pot2).unwrap();
        let duty1 = utils::map(pot1_val.into(), POT_MIN, POT_MAX, 0, max_duty);
        let duty2 = utils::map(pot2_val.into(), POT_MIN, POT_MAX, 0, max_duty);
        pwm_pin1.set_duty(duty1.into())?;
        let rpm1 = get_motor_rpm();
        println!("duty = {}%, speed = {} rpm", (100 * duty1 / max_duty), rpm1);
    }
}

// Calculates motor speed by doing an atomic read of
// the elapsed time between encoder interrupts
fn get_motor_rpm() -> f32 {
    let mut rpm: f32;
    if ENC_INTERRUPT_FLAG.load(Ordering::SeqCst) {
        let elapsed = ELAPSED_TIME_ATOMIC.load(Ordering::SeqCst);
        if elapsed == 0 {
            rpm = 0.0
        } else {
            rpm = 1.0 / (elapsed as f32);
            rpm = rpm * 1000000.0 * 60.0 / GEAR_RATIO as f32 / ENCODER_MULT as f32;
        }
        ENC_INTERRUPT_FLAG.store(false, Ordering::SeqCst);
    } else {
        rpm = 0.0;
    }

    rpm
}
