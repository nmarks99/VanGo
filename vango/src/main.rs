// basics
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys;

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
// use neopixel::Rgb;
use neopixel::Neopixel;
mod utils;

const GEAR_RATIO: u32 = 50; // TODO: check this
const ENCODER_MULT: u32 = 7;

// these values were obtained experimentally
const POT_MIN: u32 = 128;
const POT_MAX: u32 = 3139;

static LAST_TIME_ATOMIC1: AtomicU32 = AtomicU32::new(0);
static ELAPSED_TIME_ATOMIC1: AtomicU32 = AtomicU32::new(0);
static ENC_INTERRUPT_FLAG1: AtomicBool = AtomicBool::new(false);

static LAST_TIME_ATOMIC2: AtomicU32 = AtomicU32::new(0);
static ELAPSED_TIME_ATOMIC2: AtomicU32 = AtomicU32::new(0);
static ENC_INTERRUPT_FLAG2: AtomicBool = AtomicBool::new(false);

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    unsafe {
        esp_idf_sys::esp_task_wdt_delete(esp_idf_sys::xTaskGetIdleTaskHandleForCPU(
            esp_idf_hal::cpu::core() as u32,
        ));
    }
    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio12, peripherals.rmt.channel0)?;
    neo.set_color("cyan", 0.2)?;

    // system timer to get uptime
    let sys_timer1 = EspSystemTime {};
    let sys_timer2 = EspSystemTime {};
    // let sys_timer3 = EspSystemTime {};

    // configure PWM on GPIO15 for motor 1
    let mut pwm_pin1 = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio15,
    )?;

    // configure PWM on GPIO23 for motor 1
    let mut pwm_pin2 = LedcDriver::new(
        peripherals.ledc.channel1,
        LedcTimerDriver::new(
            peripherals.ledc.timer1,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio23,
    )?;

    // Sets motor direction
    let mut motor1_dir = PinDriver::output(peripherals.pins.gpio32)?;
    motor1_dir.set_low()?;
    let mut motor2_dir = PinDriver::output(peripherals.pins.gpio21)?;
    motor2_dir.set_low()?;

    let max_duty = pwm_pin1.get_max_duty(); // max duty should be the same for both

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

    // set up interrupt on enc A for first motor
    let mut m1_enc_driver = PinDriver::input(peripherals.pins.gpio27)?;
    m1_enc_driver.set_pull(gpio::Pull::Up)?;
    m1_enc_driver.set_interrupt_type(gpio::InterruptType::AnyEdge)?;

    // set up interrupt on enc A for second motor
    let mut m2_enc_driver = PinDriver::input(peripherals.pins.gpio14)?;
    m2_enc_driver.set_pull(gpio::Pull::Up)?;
    m2_enc_driver.set_interrupt_type(gpio::InterruptType::AnyEdge)?;

    // ISR for motor 1 encoder
    unsafe {
        m1_enc_driver.subscribe(move || {
            ENC_INTERRUPT_FLAG1.store(true, Ordering::SeqCst);
            let current_time = sys_timer1.now().as_micros() as u32;
            let last_time = LAST_TIME_ATOMIC1.load(Ordering::SeqCst);
            if last_time < current_time {
                ELAPSED_TIME_ATOMIC1.store(current_time - last_time, Ordering::SeqCst);
            }
            LAST_TIME_ATOMIC1.store(current_time, Ordering::SeqCst);
        })?;
    }

    // ISR for motor 2 encoder
    unsafe {
        m2_enc_driver.subscribe(move || {
            ENC_INTERRUPT_FLAG2.store(true, Ordering::SeqCst);
            let current_time = sys_timer2.now().as_micros() as u32;
            let last_time = LAST_TIME_ATOMIC2.load(Ordering::SeqCst);
            if last_time < current_time {
                ELAPSED_TIME_ATOMIC2.store(current_time - last_time, Ordering::SeqCst);
            }
            LAST_TIME_ATOMIC2.store(current_time, Ordering::SeqCst);
        })?;
    }

    // const kp: f32 = 1.0;
    // const ki: f32 = 0.1;
    // const kd: f32 = 0.1;
    // const target: f32 = 59.0;
    // let mut last_err: f32 = 0.0;
    // let mut i_err: f32 = 0.0;
    // loop {
    //     let pot1_val = adc_driver.read(&mut pot1).unwrap();
    //     let rpm = get_motor1_rpm();

    //     // begin PID loop when we turn up the pot
    //     if pot1_val >= 1000 {
    //         let err = target - rpm;
    //         let d_err = err - last_err;
    //         i_err = i_err + err;
    //         println!("{:.2}", rpm); // print current speed

    //         // compute control signal (duty cycle)
    //         let mut u: u32 = (kp*err + kd*d_err + ki*i_err) as u32;
    //         if u > max_duty {
    //             u = max_duty;
    //         }
    //         pwm_pin1.set_duty(u as u32)?;

    //         last_err = err;
    //     }
    //     else {
    //         i_err = 0.0;
    //         pwm_pin1.set_duty(0)?;
    //     }

    //     FreeRtos::delay_us(400);
    // }

    loop {
        // Get pot values
        let pot1_val = adc_driver.read(&mut pot1).unwrap();
        let pot2_val = adc_driver.read(&mut pot2).unwrap();

        // map pot value to duty cycle
        let duty1 = utils::map(pot1_val.into(), POT_MIN, POT_MAX, 0, max_duty);
        let duty2 = utils::map(pot2_val.into(), POT_MIN, POT_MAX, 0, max_duty);

        // set pwm (speed) for both motors
        pwm_pin1.set_duty(duty1.into())?;
        pwm_pin2.set_duty(duty2.into())?;

        // Get the RPM of each motor
        let rpm1 = get_motor1_rpm();
        let rpm2 = get_motor2_rpm();

        // print adc values and speeds
        println!(
            "Motor 1: {:.2}, {:.2} rpm \t Motor 2: {:.2}, {:.2} rpm",
            pot1_val, rpm1, pot2_val, rpm2
        );
        FreeRtos::delay_ms(50);
    }
}

// Calculates motor speed by doing an atomic read of
// the elapsed time between encoder interrupts
fn get_motor1_rpm() -> f32 {
    let mut rpm: f32;
    if ENC_INTERRUPT_FLAG1.load(Ordering::SeqCst) {
        let elapsed = ELAPSED_TIME_ATOMIC1.load(Ordering::SeqCst);
        if elapsed == 0 {
            rpm = 0.0
        } else {
            rpm = 1.0 / (elapsed as f32);
            rpm = rpm * 1000000.0 * 60.0 / GEAR_RATIO as f32 / ENCODER_MULT as f32;
        }
        ENC_INTERRUPT_FLAG1.store(false, Ordering::SeqCst);
    } else {
        rpm = 0.0;
    }

    rpm
}

fn get_motor2_rpm() -> f32 {
    let mut rpm: f32;
    if ENC_INTERRUPT_FLAG2.load(Ordering::SeqCst) {
        let elapsed = ELAPSED_TIME_ATOMIC2.load(Ordering::SeqCst);
        if elapsed == 0 {
            rpm = 0.0
        } else {
            rpm = 1.0 / (elapsed as f32);
            rpm = rpm * 1000000.0 * 60.0 / GEAR_RATIO as f32 / ENCODER_MULT as f32;
        }
        ENC_INTERRUPT_FLAG2.store(false, Ordering::SeqCst);
    } else {
        rpm = 0.0;
    }

    rpm
}
