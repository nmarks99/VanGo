// basics
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicI32, Ordering};
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
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_hal::task::executor::{Monitor, Notify, Wait, FreeRtosMonitor};

// user modules
mod neopixel;
use neopixel::Neopixel;
mod utils;
mod motor;
mod encoder;
use encoder::Encoder;


// these values were obtained experimentally
const POT_MIN: u32 = 128;
const POT_MAX: u32 = 3139;

const ENCODER_RATE_MS: u64 = 10; // 100 Hz
const GEAR_RATIO: u64 = 50; // TODO: check this
const ENCODER_MULT: u64 = 7;
const TICKS_TO_RPS: f32 = 1000.0 / (ENCODER_MULT * GEAR_RATIO * ENCODER_RATE_MS * 4) as f32;
const TICKS_TO_RPM: f32 = 60.0*1000.0 / (ENCODER_MULT * GEAR_RATIO * ENCODER_RATE_MS * 4) as f32;

static LAST_COUNT1: AtomicI32 = AtomicI32::new(0);
static LAST_COUNT2: AtomicI32 = AtomicI32::new(0);
static RPM1: AtomicU32 = AtomicU32::new(0);
static RPM2: AtomicU32 = AtomicU32::new(0);
// static LAST_TIME: AtomicU32 = AtomicU32::new(0);

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

    // system timer to get uptime
    // let sys_timer1 = EspSystemTime {};
    // let sys_timer2 = EspSystemTime {};
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

    // set up encoder for each motor
    let encoder1 = Encoder::new(
        peripherals.pcnt0,
        peripherals.pins.gpio27,
        peripherals.pins.gpio33
    )?;
    let encoder2 = Encoder::new(
        peripherals.pcnt1,
        peripherals.pins.gpio14,
        peripherals.pins.gpio22
    )?;

    // Make a task for computing motor speed
    let task_timer = esp_idf_svc::timer::EspTaskTimerService::new().unwrap();
    let monitor = FreeRtosMonitor::new();
    let monitor_notify = monitor.notifier();

    // ISR (kinda?)
    let task_timer = task_timer
        .timer(move || {
            monitor_notify.notify();

            let count1 = encoder1.get_value().unwrap() as i32;
            let last_count1 = LAST_COUNT1.load(Ordering::SeqCst) as i32;
            let rpm1: f32 = (count1-last_count1).abs() as f32 * TICKS_TO_RPM;
            RPM1.store(rpm1 as u32, Ordering::SeqCst);
            LAST_COUNT1.store(count1,Ordering::SeqCst);

            let count2 = encoder2.get_value().unwrap() as i32;
            let last_count2 = LAST_COUNT2.load(Ordering::SeqCst) as i32;
            let rpm2: f32 = (count2-last_count2).abs() as f32 * TICKS_TO_RPM;
            RPM2.store(rpm2 as u32, Ordering::SeqCst);
            LAST_COUNT2.store(count2,Ordering::SeqCst);

        })
        .unwrap();
    task_timer.every(Duration::from_millis(ENCODER_RATE_MS)).unwrap();

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
        let rpm1 = RPM1.load(Ordering::SeqCst);
        println!("RPM1 = {} rpm", rpm1);
        let rpm2 = RPM2.load(Ordering::SeqCst);
        println!("RPM2 = {} rpm", rpm2);
        println!("-------------");

        FreeRtos::delay_ms(50);
    }
}
