// basics
use core::sync::atomic::{AtomicI16, AtomicI32, Ordering};
use core::time::Duration;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;

// use diff_drive::rigid2d::Vector2D;

// GPIO
use esp_idf_hal::gpio::Level;
use esp_idf_hal::gpio::PinDriver;

// LEDC (PWM)
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::LedcTimerDriver;

// TIME
use esp_idf_hal::task::executor::{FreeRtosMonitor, Monitor, Notify};
// use esp_idf_svc::systime::EspSystemTime;
// static SYS_TIMER: EspSystemTime = EspSystemTime {};

// BLE
use esp32_nimble::utilities::BleUuid;
use esp32_nimble::{uuid128, BLEDevice, NimbleProperties};

// local modules
mod encoder;
mod neopixel;
mod pen;
use encoder::Encoder;
use encoder::{ENCODER_RATE_MS, TICKS_TO_RPM};
use neopixel::Neopixel;
use vango_utils as utils;
// use pen::{Pen, PenState};

// Measured encoder counts and speeds set in timer interrupt
static LEFT_COUNT: AtomicI32 = AtomicI32::new(0);
static RIGHT_COUNT: AtomicI32 = AtomicI32::new(0);
static LEFT_RPM: AtomicI16 = AtomicI16::new(0);
static RIGHT_RPM: AtomicI16 = AtomicI16::new(0);

// Target speeds set by BLE callbacks
static TARGET_RPM_LEFT: AtomicI16 = AtomicI16::new(0);
static TARGET_RPM_RIGHT: AtomicI16 = AtomicI16::new(0);

// BLE UUIDs
const VANGO_SERVICE_UUID: BleUuid = uuid128!("21470560-232e-11ee-be56-0242ac120002");
const LEFT_SPEED_UUID: BleUuid = uuid128!("3c9a3f00-8ed3-4bdf-8a39-a01bebede295");
const RIGHT_SPEED_UUID: BleUuid = uuid128!("c0ffc89c-29bb-11ee-be56-0242ac120002");
const LEFT_COUNTS_UUID: BleUuid = uuid128!("0a286b70-2c2b-11ee-be56-0242ac120002");
const RIGHT_COUNTS_UUID: BleUuid = uuid128!("0a28672e-2c2b-11ee-be56-0242ac120002");
const WAYPOINT_UUID: BleUuid = uuid128!("21e16dea-357a-11ee-be56-0242ac120002");

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    esp_idf_svc::log::EspLogger::initialize_default();

    // let mut waypoints_vec: Vec<>

    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio21, peripherals.rmt.channel0)?;
    neo.set_color("blue", 0.2)?;

    // Setup BLE server
    let ble_device = BLEDevice::take();
    let server = ble_device.get_server();
    server.on_connect(|server, desc| {
        log::info!("Client connected");
        ble_device.get_advertising().start().unwrap();
        server
            .update_conn_params(desc.conn_handle, 6, 12, 0, 60)
            .unwrap()
    });
    let ble_service = server.create_service(VANGO_SERVICE_UUID);

    // BLE characteristic for waypoints
    let waypoint_blec = ble_service
        .lock()
        .create_characteristic(WAYPOINT_UUID, NimbleProperties::WRITE);
    waypoint_blec.lock().on_write(move |recv, _param| {
        let waypoint_bytes = recv;
        log::info!("Waypoint: {:?}", waypoint_bytes);
    });

    // BLE characteristic for left motor speed
    let left_speed_blec = ble_service.lock().create_characteristic(
        LEFT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    left_speed_blec
        .lock()
        .on_read(move |v, _| {
            let speed = LEFT_RPM.load(Ordering::Relaxed);
            v.set_value(&utils::int_to_bytes(speed));
            log::info!("Left speed = {:?}", speed);
        })
        .on_write(move |recv, _param| {
            let rpm_target_recv: i16 = utils::bytes_to_int(recv).unwrap();
            log::info!("Left speed target {:?}", rpm_target_recv);
            TARGET_RPM_LEFT.store(rpm_target_recv, Ordering::SeqCst);
        });

    // BLE characteristic for right motor speed
    let right_speed_blec = ble_service.lock().create_characteristic(
        RIGHT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    right_speed_blec
        .lock()
        .on_read(move |v, _| {
            let speed = RIGHT_RPM.load(Ordering::Relaxed);
            v.set_value(&utils::int_to_bytes(speed));
            log::info!("Right speed = {:?}", speed);
        })
        .on_write(move |recv, _param| {
            let rpm_target_recv: i16 = utils::bytes_to_int(recv).unwrap();
            log::info!("Right speed target {:?}", rpm_target_recv);
            TARGET_RPM_RIGHT.store(rpm_target_recv, Ordering::SeqCst);
        });

    // BLE characteristics for reading right encoder counts
    let right_counts_blec = ble_service
        .lock()
        .create_characteristic(RIGHT_COUNTS_UUID, NimbleProperties::READ);
    right_counts_blec.lock().on_read(move |v, _| {
        let counts = RIGHT_COUNT.load(Ordering::Relaxed);
        v.set_value(&utils::int_to_bytes(counts));
        log::info!("Right counts read: {:?}", counts);
    });

    // BLE characteristics for reading left encoder counts
    let left_counts_blec = ble_service
        .lock()
        .create_characteristic(LEFT_COUNTS_UUID, NimbleProperties::READ);
    left_counts_blec.lock().on_read(move |v, _| {
        let counts = LEFT_COUNT.load(Ordering::Relaxed);
        v.set_value(&utils::int_to_bytes(counts));
        log::info!("Left counts read: {:?}", counts);
    });

    // start BLE advertising
    let ble_advertising = ble_device.get_advertising();
    ble_advertising
        .name("VanGo")
        .add_service_uuid(VANGO_SERVICE_UUID);
    ble_advertising.start().unwrap();

    // configure PWM on GPIO15 for motor 1
    let mut left_pwm_driver = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio13,
    )?;

    // configure PWM on GPIO23 for motor 2
    let mut right_pwm_driver = LedcDriver::new(
        peripherals.ledc.channel1,
        LedcTimerDriver::new(
            peripherals.ledc.timer1,
            &TimerConfig::new().frequency(80.Hz().into()),
        )?,
        peripherals.pins.gpio17,
    )?;

    // Set up Pen
    // let mut pen = Pen::new(
    //     peripherals.pins.gpio15,
    //     peripherals.ledc.channel2,
    //     peripherals.ledc.timer2,
    // )?;

    // Sets motor direction
    let mut left_direction = PinDriver::output(peripherals.pins.gpio12)?;
    left_direction.set_low()?;
    let mut right_direction = PinDriver::output(peripherals.pins.gpio32)?;
    right_direction.set_low()?;

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

    // Make a task (timer interrupt) for computing motor speed
    let task_timer = esp_idf_svc::timer::EspTaskTimerService::new().unwrap();
    let monitor = FreeRtosMonitor::new();
    let monitor_notify = monitor.notifier();

    // Timer based ISR
    let task_timer = task_timer
        .timer(move || {
            monitor_notify.notify(); // not sure what this does

            // Get left counts and RPM
            let count1 = left_encoder.get_value().unwrap() as i32;
            let last_count1 = LEFT_COUNT.load(Ordering::SeqCst) as i32;
            let left_speed: f32 = (count1 - last_count1).abs() as f32 * TICKS_TO_RPM;
            LEFT_RPM.store(left_speed as i16, Ordering::SeqCst);
            LEFT_COUNT.store(count1, Ordering::SeqCst);

            // Get right counts and RPM
            let count2 = right_encoder.get_value().unwrap() as i32;
            let last_count2 = RIGHT_COUNT.load(Ordering::SeqCst) as i32;
            let right_speed: f32 = (count2 - last_count2).abs() as f32 * TICKS_TO_RPM;
            RIGHT_RPM.store(right_speed as i16, Ordering::SeqCst);
            RIGHT_COUNT.store(count2, Ordering::SeqCst);
        })
        .unwrap();

    task_timer
        .every(Duration::from_millis(ENCODER_RATE_MS))
        .unwrap();

    // use control::PidController;
    // let mut left_pid = PidController::new(1.0, 0.0, 0.0);
    // let mut right_pid = PidController::new(1.0, 0.0, 0.0);

    loop {
        // Get target speeds which are set in BLE callback
        let target_left = TARGET_RPM_LEFT.load(Ordering::SeqCst);
        let target_right = TARGET_RPM_RIGHT.load(Ordering::SeqCst);

        // Set direction based on sign of target speed
        let left_dir = if target_left < 0 {
            Level::High
        } else {
            Level::Low
        };

        let right_dir = if target_right > 0 {
            Level::High
        } else {
            Level::Low
        };
        left_direction.set_level(left_dir)?;
        right_direction.set_level(right_dir)?;

        // Get the RPM of each motor
        // let left_speed = LEFT_RPM.load(Ordering::SeqCst);
        // let right_speed = RIGHT_RPM.load(Ordering::SeqCst);

        // Compute control signal to send from PID controller
        // TODO: Check if this makes sense
        // let u1 = left_pid.compute(target_left as f32, left_speed as f32);
        // let u2 = right_pid.compute(target_right as f32, right_speed as f32);

        // Set duty cycle for each motor
        left_pwm_driver.set_duty(target_left.abs() as u32)?;
        right_pwm_driver.set_duty(target_right.abs() as u32)?;

        // println!("-------------");
        // println!("Motor 1:");
        // println!(
        //     "measured = {}\ntarget = {}\nu = {:.3}\n",
        //     left_speed, target_left, u1
        // );
        // println!("-------------");
        // println!("Motor 2:");
        // println!(
        //     "measured = {}\ntarget = {}\nu = {:.3}",
        //     right_speed, target_right, u2
        // );

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
