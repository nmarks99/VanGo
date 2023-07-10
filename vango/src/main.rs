// basics
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use core::time::Duration;
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
use esp_idf_hal::task::executor::{FreeRtosMonitor, Monitor, Notify};
use esp_idf_svc::systime::EspSystemTime;

// user modules
mod encoder;
mod neopixel;
mod pen;
mod utils;
use encoder::Encoder;
use encoder::{ENCODER_RATE_MS, TICKS_TO_RPM};
use neopixel::Neopixel;
use pen::{Pen, PenState};

const POT_MIN: u16 = 128;
const POT_MAX: u16 = 3139;

static LAST_COUNT_LEFT: AtomicI32 = AtomicI32::new(0);
static LAST_COUNT_RIGHT: AtomicI32 = AtomicI32::new(0);
static LEFT_SPEED: AtomicU32 = AtomicU32::new(0);
static RIGHT_SPEED: AtomicU32 = AtomicU32::new(0);

#[allow(dead_code)]
static SYS_TIMER: EspSystemTime = EspSystemTime {};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();

    // disable watchdog, not sure if this works though
    unsafe {
        esp_idf_sys::esp_task_wdt_delete(esp_idf_sys::xTaskGetIdleTaskHandleForCPU(
            esp_idf_hal::cpu::core() as u32,
        ));
    }

    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio21, peripherals.rmt.channel0)?;
    neo.set_color("red", 0.2)?;

    // configure PWM on GPIO15 for motor 1
    let mut left_pwm_driver = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio13,
    )?;

    // configure PWM on GPIO23 for motor 1
    let mut right_pwm_driver = LedcDriver::new(
        peripherals.ledc.channel1,
        LedcTimerDriver::new(
            peripherals.ledc.timer1,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio17,
    )?;

    // Set up Pen
    let mut pen = Pen::new(
        peripherals.pins.gpio15,
        peripherals.ledc.channel2,
        peripherals.ledc.timer2,
    )?;

    // Sets motor direction
    let mut left_direction = PinDriver::output(peripherals.pins.gpio12)?;
    left_direction.set_low()?;
    let mut right_direction = PinDriver::output(peripherals.pins.gpio32)?;
    right_direction.set_low()?;

    // let max_duty = left_pwm_driver.get_max_duty(); // max duty should be the same for both

    // Setup adc driver
    let mut adc_driver = AdcDriver::new(
        peripherals.adc2,
        &adc::config::Config::new().calibration(true),
    )?;

    // setup analog input for pot_left and pot_right
    let mut pot_left: AdcChannelDriver<'_, gpio::Gpio26, Atten11dB<_>> =
        adc::AdcChannelDriver::new(peripherals.pins.gpio26)?;
    let mut pot_right: AdcChannelDriver<'_, gpio::Gpio25, Atten11dB<_>> =
        adc::AdcChannelDriver::new(peripherals.pins.gpio25)?;

    // set up encoder for each motor
    let left_encoder = Encoder::new(
        peripherals.pcnt0,
        peripherals.pins.gpio27,
        peripherals.pins.gpio33,
    )?;
    let right_encoder = Encoder::new(
        peripherals.pcnt1,
        peripherals.pins.gpio14,
        peripherals.pins.gpio16,
    )?;

    // Make a task for computing motor speed
    let task_timer = esp_idf_svc::timer::EspTaskTimerService::new().unwrap();
    let monitor = FreeRtosMonitor::new();
    let monitor_notify = monitor.notifier();

    // Timer based ISR
    let task_timer = task_timer
        .timer(move || {
            monitor_notify.notify(); // not sure what this does

            let count1 = left_encoder.get_value().unwrap() as i32;
            let last_count1 = LAST_COUNT_LEFT.load(Ordering::SeqCst) as i32;
            let left_speed: f32 = (count1 - last_count1).abs() as f32 * TICKS_TO_RPM;
            LEFT_SPEED.store(left_speed as u32, Ordering::SeqCst);
            LAST_COUNT_LEFT.store(count1, Ordering::SeqCst);

            let count2 = right_encoder.get_value().unwrap() as i32;
            let last_count2 = LAST_COUNT_RIGHT.load(Ordering::SeqCst) as i32;
            let right_speed: f32 = (count2 - last_count2).abs() as f32 * TICKS_TO_RPM;
            RIGHT_SPEED.store(right_speed as u32, Ordering::SeqCst);
            LAST_COUNT_RIGHT.store(count2, Ordering::SeqCst);
        })
        .unwrap();

    task_timer
        .every(Duration::from_millis(ENCODER_RATE_MS))
        .unwrap();

    use control::PidController;
    let mut left_pid = PidController::new(1.0, 0.1, 0.0);
    let mut right_pid = PidController::new(1.0, 0.1, 0.0);

    // let mut state: PenState;
    loop {
        // pen.up()?;
        // FreeRtos::delay_ms(2000);
        // state = pen.get_state()?;
        // println!("State = {:?}", state);
        // pen.down()?;
        // FreeRtos::delay_ms(2000);
        // state = pen.get_state()?;
        // println!("State = {:?}", state);

        // Get pot values
        let pot_left_val = adc_driver.read(&mut pot_left).unwrap();
        let pot_right_val = adc_driver.read(&mut pot_right).unwrap();
        println!("pot left = {pot_left_val}");
        println!("pot right = {pot_right_val}");

        // map pot value to duty cycle
        let target_left = utils::map(pot_left_val, POT_MIN, POT_MAX, 0u16, 257u16);
        let target_right = utils::map(pot_right_val, POT_MIN, POT_MAX, 0u16, 270u16);
        println!("target left = {target_left}");
        println!("target right = {target_right}");

        // Get the RPM of each motor
        let left_speed = LEFT_SPEED.load(Ordering::SeqCst);
        let right_speed = RIGHT_SPEED.load(Ordering::SeqCst);

        // Compute control signal to send from PID controller
        let u1 = left_pid.compute(target_left as f32, left_speed as f32);
        let u2 = right_pid.compute(target_right as f32, right_speed as f32);

        // Set duty cycle for each motor
        left_pwm_driver.set_duty(u1 as u32)?;
        right_pwm_driver.set_duty(u2 as u32)?;

        println!("-------------");
        println!("Motor 1:");
        println!(
            "measured = {}\ntarget = {}\nu = {:.3}",
            left_speed, target_left, u1
        );
        println!("-------------");
        println!("Motor 2:");
        println!(
            "measured = {}\ntarget = {}\nu = {:.3}",
            right_speed, target_right, u2
        );

        FreeRtos::delay_ms(15);
    }
}

mod control {
    #![allow(dead_code)]
    pub struct PidController {
        target: f32,
        kp: f32,
        ki: f32,
        kd: f32,
        err: f32,
        i_err: f32,
        d_err: f32,
        last_err: f32,
    }

    impl PidController {
        pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
            PidController {
                target: 0.0,
                kp,
                ki,
                kd,
                err: 0.0,
                i_err: 0.0,
                d_err: 0.0,
                last_err: 0.0,
            }
        }

        pub fn set_kp(&mut self, kp: f32) {
            self.kp = kp;
        }

        pub fn set_ki(&mut self, ki: f32) {
            self.ki = ki;
        }

        pub fn set_kd(&mut self, kd: f32) {
            self.kd = kd;
        }

        pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
            self.kp = kp;
            self.ki = ki;
            self.kd = kd;
        }

        pub fn compute(&mut self, target: f32, current: f32) -> f32 {
            self.target = target;
            self.err = self.target - current;
            self.d_err = self.err - self.last_err;

            // only accumulate integral error when error is small
            if self.err < 20.0 {
                self.i_err = self.i_err + self.err;
            } else {
                self.i_err = 0.0;
            }

            let u = self.kp * self.err + self.ki * self.i_err + self.kd * self.d_err;
            u
        }
    }
}
