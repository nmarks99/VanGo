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
mod neopixel;
use neopixel::Neopixel;
mod encoder;
mod utils;
use encoder::Encoder;
use encoder::{ENCODER_RATE_MS, TICKS_TO_RPM};

const POT_MIN: u32 = 128;
const POT_MAX: u32 = 3139;

static LAST_COUNT1: AtomicI32 = AtomicI32::new(0);
static LAST_COUNT2: AtomicI32 = AtomicI32::new(0);
static SPEED1: AtomicU32 = AtomicU32::new(0);
static SPEED2: AtomicU32 = AtomicU32::new(0);

static SYS_TIMER: EspSystemTime = EspSystemTime {};

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
    neo.set_color("red", 0.2)?;

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

    // set up encoder for each motor
    let encoder1 = Encoder::new(
        peripherals.pcnt0,
        peripherals.pins.gpio27,
        peripherals.pins.gpio33,
    )?;
    let encoder2 = Encoder::new(
        peripherals.pcnt1,
        peripherals.pins.gpio14,
        peripherals.pins.gpio22,
    )?;

    // Make a task for computing motor speed
    let task_timer = esp_idf_svc::timer::EspTaskTimerService::new().unwrap();
    let monitor = FreeRtosMonitor::new();
    let monitor_notify = monitor.notifier();

    // Timer based ISR
    let task_timer = task_timer
        .timer(move || {
            monitor_notify.notify(); // not sure what this does

            let count1 = encoder1.get_value().unwrap() as i32;
            let last_count1 = LAST_COUNT1.load(Ordering::SeqCst) as i32;
            let speed1: f32 = (count1 - last_count1).abs() as f32 * TICKS_TO_RPM;
            SPEED1.store(speed1 as u32, Ordering::SeqCst);
            LAST_COUNT1.store(count1, Ordering::SeqCst);

            let count2 = encoder2.get_value().unwrap() as i32;
            let last_count2 = LAST_COUNT2.load(Ordering::SeqCst) as i32;
            let speed2: f32 = (count2 - last_count2).abs() as f32 * TICKS_TO_RPM;
            SPEED2.store(speed2 as u32, Ordering::SeqCst);
            LAST_COUNT2.store(count2, Ordering::SeqCst);
        })
        .unwrap();

    task_timer
        .every(Duration::from_millis(ENCODER_RATE_MS))
        .unwrap();

    use control::PidController;
    let mut motor1_pid = PidController::new(2.0, 0.02, 0.001);
    loop {
        // Get pot values
        let pot1_val = adc_driver.read(&mut pot1).unwrap();
        // let pot2_val = adc_driver.read(&mut pot2).unwrap();

        // map pot value to duty cycle
        let duty1 = utils::map(pot1_val.into(), POT_MIN, POT_MAX, 0, max_duty);
        // let duty2 = utils::map(pot2_val.into(), POT_MIN, POT_MAX, 0, max_duty);

        // Get the RPM of each motor
        let speed1 = SPEED1.load(Ordering::SeqCst);
        // println!("RPM1 = {} rpm", speed1);
        // let speed2 = SPEED2.load(Ordering::SeqCst);
        // println!("RPM2 = {} rpm", speed2);

        let u = motor1_pid.compute(duty1 as f32, speed1 as f32);
        pwm_pin1.set_duty(u as u32)?;

        println!("now = {}\ntarget = {}\nu = {:.3}", speed1, duty1, u);
        println!("-------------");
        // set pwm (speed) for both motors
        // pwm_pin1.set_duty(duty1.into())?;
        // pwm_pin2.set_duty(duty2.into())?;

        FreeRtos::delay_ms(50);
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
