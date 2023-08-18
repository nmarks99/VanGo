// Basics
use atomic_float::AtomicF32;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use core::time::Duration;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;

// GPIO
use esp_idf_hal::gpio::PinDriver;

// LEDC (PWM)
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::ledc::LedcTimerDriver;

// TIME
use esp_idf_hal::task::executor::{FreeRtosMonitor, Monitor, Notify};
use esp_idf_svc::systime::EspSystemTime;
static SYS_TIMER: EspSystemTime = EspSystemTime {};

// BLE
use esp32_nimble::utilities::BleUuid;
use esp32_nimble::{uuid128, BLEDevice, NimbleProperties};

// diff-drive
use diff_drive::ddrive::{DiffDrive, WheelState};
use diff_drive::rigid2d::{Pose2D, Twist2D};
use diff_drive::utils::rad2deg;

// local modules
mod encoder;
mod neopixel;
mod pen;
use encoder::Encoder;
use encoder::{ENCODER_RATE_MS, TICKS_PER_RAD};
use neopixel::Neopixel;
use pen::Pen;
use vango_utils as utils;
// use pen::{Pen, PenState};

// Atomic variables
static LEFT_COUNT: AtomicI32 = AtomicI32::new(0);
static RIGHT_COUNT: AtomicI32 = AtomicI32::new(0);
static LEFT_SPEED: AtomicF32 = AtomicF32::new(0.0);
static LEFT_ANGLE: AtomicF32 = AtomicF32::new(0.0);
static RIGHT_ANGLE: AtomicF32 = AtomicF32::new(0.0);
static RIGHT_SPEED: AtomicF32 = AtomicF32::new(0.0);
static LEFT_DUTY: AtomicF32 = AtomicF32::new(0.0);
static RIGHT_DUTY: AtomicF32 = AtomicF32::new(0.0);
static ISR_FLAG: AtomicBool = AtomicBool::new(false);
static TARGET_SPEED_LEFT: AtomicF32 = AtomicF32::new(0.0);
static TARGET_SPEED_RIGHT: AtomicF32 = AtomicF32::new(0.0);
static POSE_X: AtomicF32 = AtomicF32::new(0.0);
static POSE_Y: AtomicF32 = AtomicF32::new(0.0);
static POSE_THETA: AtomicF32 = AtomicF32::new(0.0);

// BLE UUIDs
const VANGO_SERVICE_UUID: BleUuid = uuid128!("21470560-232e-11ee-be56-0242ac120002");
const LEFT_SPEED_UUID: BleUuid = uuid128!("3c9a3f00-8ed3-4bdf-8a39-a01bebede295");
const RIGHT_SPEED_UUID: BleUuid = uuid128!("c0ffc89c-29bb-11ee-be56-0242ac120002");
const WAYPOINT_UUID: BleUuid = uuid128!("21e16dea-357a-11ee-be56-0242ac120002");
const POSE_THETA_UUID: BleUuid = uuid128!("3cedc40e-3655-11ee-be56-0242ac120002");
const POSE_X_UUID: BleUuid = uuid128!("a0c2b3b2-3b1a-11ee-be56-0242ac120002");
const POSE_Y_UUID: BleUuid = uuid128!("a0c2b65a-3b1a-11ee-be56-0242ac120002");
const PEN_UUID: BleUuid = uuid128!("0daaac7c-3d6a-11ee-be56-0242ac120002");

// Speed controller
// Proportional control seems fine and is faster
const KP: f32 = 0.3;
const KD: f32 = 0.0; // 1.2
const DIR_CHANGE_THRESHOLD: f32 = 1.0;

// Robot paramaters
const WHEEL_RADIUS: f32 = 0.045 / 2.0; // meters
const WHEEL_SEPARATION: f32 = 0.140; // meters

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    esp_idf_svc::log::EspLogger::initialize_default();

    // let mut waypoints_vec: Vec<>

    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio21, peripherals.rmt.channel0)?;
    neo.set_color("purple", 0.2)?;

    // Set up Pen
    let mut pen = Pen::new(
        peripherals.pins.gpio15,
        peripherals.ledc.channel2,
        peripherals.ledc.timer2,
    )?;
    pen.up()?;

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
    waypoint_blec.lock().on_write(move |recv| {
        let waypoint_bytes = recv.recv_data;
        log::info!("Waypoint: {:?}", waypoint_bytes);
    });

    let pen_blec = ble_service
        .lock()
        .create_characteristic(PEN_UUID, NimbleProperties::WRITE | NimbleProperties::READ);
    pen_blec.lock().on_write(move |recv| {
        let v = recv.recv_data;
        if v.len() == 1 {
            if v[0] == b'1' {
                let _ = pen.up();
            } else if v[0] == b'0' {
                let _ = pen.down();
            }
        } else {
            log::error!("Invalid input to pen ble characteristic ")
        }
    });

    // BLE characteristics for pose estimate from odometry
    let pose_x_blec = ble_service
        .lock()
        .create_characteristic(POSE_X_UUID, NimbleProperties::READ);
    pose_x_blec.lock().on_read(move |v, _| {
        let pose_x = POSE_X.load(Ordering::Acquire);
        v.set_value(&utils::f32_to_ascii(pose_x));
    });
    let pose_y_blec = ble_service
        .lock()
        .create_characteristic(POSE_Y_UUID, NimbleProperties::READ);
    pose_y_blec.lock().on_read(move |v, _| {
        let pose_y = POSE_Y.load(Ordering::Acquire);
        v.set_value(&utils::f32_to_ascii(pose_y));
    });
    let pose_theta_blec = ble_service
        .lock()
        .create_characteristic(POSE_THETA_UUID, NimbleProperties::READ);
    pose_theta_blec.lock().on_read(move |v, _| {
        let pose_theta = POSE_THETA.load(Ordering::Acquire);
        v.set_value(&utils::f32_to_ascii(pose_theta));
    });

    // BLE characteristic for left motor speed
    let left_speed_blec = ble_service.lock().create_characteristic(
        LEFT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    left_speed_blec
        .lock()
        .on_read(move |v, _| {
            let speed = LEFT_SPEED.load(Ordering::Acquire);
            v.set_value(&utils::f32_to_ascii(speed));
        })
        .on_write(move |recv| {
            let target_recv: f32 = utils::ascii_to_f32(recv.recv_data.to_vec()).unwrap();
            TARGET_SPEED_LEFT.store(target_recv, Ordering::Release);
        });

    // BLE characteristic for right motor speed
    let right_speed_blec = ble_service.lock().create_characteristic(
        RIGHT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    right_speed_blec
        .lock()
        .on_read(move |v, _| {
            let speed = RIGHT_SPEED.load(Ordering::Acquire);
            v.set_value(&utils::f32_to_ascii(speed));
        })
        .on_write(move |recv| {
            let target_recv: f32 = utils::ascii_to_f32(recv.recv_data.to_vec()).unwrap();
            TARGET_SPEED_RIGHT.store(target_recv, Ordering::Release);
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

    // Set initial motor directions to forward
    let mut left_direction = PinDriver::output(peripherals.pins.gpio12)?;
    let mut right_direction = PinDriver::output(peripherals.pins.gpio32)?;
    left_direction.set_low()?;
    right_direction.set_high()?;
    let max_duty = right_pwm_driver.get_max_duty();

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

    const ALPHA: f32 = 0.1; // used in low-pass filter
    let mut left_speed_smooth = 0.0;
    let mut right_speed_smooth = 0.0;
    let mut left_err_last = 0.0;
    let mut right_err_last = 0.0;

    // Timer based ISR
    let task_timer = task_timer
        .timer(move || {
            monitor_notify.notify(); // not sure what this does
            ISR_FLAG.store(true, Ordering::SeqCst);

            // Get counts and fix angle
            let left_count = -left_encoder.get_value().unwrap() as i32;
            let right_count = -right_encoder.get_value().unwrap() as i32;

            // Compute angles
            let left_angle = left_count as f32 / TICKS_PER_RAD;
            let right_angle = right_count as f32 / TICKS_PER_RAD;

            // Compute speeds
            let left_count_last = LEFT_COUNT.load(Ordering::SeqCst);
            let mut left_speed = (left_count as f32 - left_count_last as f32)
                / (TICKS_PER_RAD * ENCODER_RATE_MS as f32 / 1000.0);
            let right_count_last = RIGHT_COUNT.load(Ordering::SeqCst);
            let mut right_speed = (right_count as f32 - right_count_last as f32)
                / (TICKS_PER_RAD * ENCODER_RATE_MS as f32 / 1000.0);

            // Low-pass filter the speed to remove high frequency noise
            left_speed_smooth = ALPHA * left_speed + (1.0 - ALPHA) * left_speed_smooth;
            right_speed_smooth = ALPHA * right_speed + (1.0 - ALPHA) * right_speed_smooth;
            left_speed = left_speed_smooth;
            right_speed = right_speed_smooth;

            // Store the counts, angles(rad), and speeds(rad/s)
            LEFT_COUNT.store(left_count, Ordering::SeqCst);
            RIGHT_COUNT.store(right_count, Ordering::SeqCst);
            LEFT_ANGLE.store(left_angle, Ordering::SeqCst);
            RIGHT_ANGLE.store(right_angle, Ordering::SeqCst);
            LEFT_SPEED.store(left_speed, Ordering::SeqCst);
            RIGHT_SPEED.store(right_speed, Ordering::SeqCst);

            // Load target speeds
            let mut target_speed_left = TARGET_SPEED_LEFT.load(Ordering::SeqCst);
            let mut target_speed_right = TARGET_SPEED_RIGHT.load(Ordering::SeqCst);

            // prevent abrupt direction changes
            // probably should just got immediately to zero then change direction
            if utils::opposite_signs(target_speed_left, left_speed)
                && left_speed.abs() > DIR_CHANGE_THRESHOLD
            {
                target_speed_left = if left_speed > 0.0 {
                    DIR_CHANGE_THRESHOLD
                } else {
                    -DIR_CHANGE_THRESHOLD
                };
            }
            if utils::opposite_signs(target_speed_right, right_speed)
                && right_speed.abs() > DIR_CHANGE_THRESHOLD
            {
                target_speed_right = if right_speed > 0.0 {
                    DIR_CHANGE_THRESHOLD
                } else {
                    -DIR_CHANGE_THRESHOLD
                };
            }

            // Compute the error and derivative error
            let err_left = target_speed_left - left_speed;
            let err_right = target_speed_right - right_speed;
            let drv_err_left = err_left - left_err_last;
            let drv_err_right = err_right - right_err_last;

            // Compute the control signal (PID controller)
            let mut u_left = KP * err_left + KD * drv_err_left;
            let mut u_right = KP * err_right + KD * drv_err_left;

            left_err_last = err_left;
            right_err_last = err_right;

            // Adjust control signal based on sign of target speed
            if target_speed_left < 0.0 {
                // Backward
                u_left = -u_left;
                let _ = left_direction.set_high();
            } else if target_speed_left > 0.0 {
                // Forward
                let _ = left_direction.set_low();
            } else {
                if left_speed > 0.0 {
                    let _ = left_direction.set_low();
                } else if left_speed < 0.0 {
                    u_left = -u_left;
                    let _ = left_direction.set_high();
                }
            }
            if target_speed_right < 0.0 {
                // Backward
                u_right = -u_right;
                let _ = right_direction.set_low();
            } else if target_speed_right > 0.0 {
                // Forward
                let _ = right_direction.set_high();
            } else {
                if right_speed > 0.0 {
                    let _ = right_direction.set_high();
                } else if right_speed < 0.0 {
                    u_right = -u_right;
                    let _ = right_direction.set_low();
                }
            }

            // load the last duty cycle value and compute new duty cycle
            let mut left_duty = LEFT_DUTY.load(Ordering::SeqCst);
            let mut right_duty = RIGHT_DUTY.load(Ordering::SeqCst);
            left_duty = left_duty + u_left;
            if left_duty > max_duty as f32 {
                left_duty = max_duty as f32;
            } else if left_duty < 0.0 {
                left_duty = 0.0;
            }
            right_duty = right_duty + u_right;
            if right_duty > max_duty as f32 {
                right_duty = max_duty as f32;
            } else if right_duty < 0.0 {
                right_duty = 0.0;
            }

            LEFT_DUTY.store(left_duty, Ordering::SeqCst);
            RIGHT_DUTY.store(right_duty, Ordering::SeqCst);

            // Set the motor to this duty cycle
            if target_speed_left.abs() > DIR_CHANGE_THRESHOLD {
                let _ = left_pwm_driver.set_duty(left_duty as u32);
            } else {
                let _ = left_pwm_driver.set_duty(0u32);
            }
            if target_speed_right.abs() > DIR_CHANGE_THRESHOLD {
                let _ = right_pwm_driver.set_duty(right_duty as u32);
            } else {
                let _ = right_pwm_driver.set_duty(0u32);
            }
        })
        .unwrap();
    task_timer
        .every(Duration::from_millis(ENCODER_RATE_MS))
        .unwrap();

    let mut robot = DiffDrive::new(WHEEL_RADIUS, WHEEL_SEPARATION);
    let colors = vec!["red", "blue", "green", "purple", "yellow", "cyan", "white"];
    let mut color_index: usize = 0;
    let mut count = 0;
    loop {
        // If the values have been updated, compute the pose from odometry
        let isr_flag = ISR_FLAG.load(Ordering::Relaxed);
        if isr_flag {
            let wheel_angles = WheelState::new(
                LEFT_ANGLE.load(Ordering::Relaxed),
                RIGHT_ANGLE.load(Ordering::Relaxed),
            );
            let pose = robot.forward_kinematics(wheel_angles);
            POSE_X.store(pose.x, Ordering::Relaxed);
            POSE_Y.store(pose.y, Ordering::Relaxed);
            POSE_THETA.store(pose.theta, Ordering::Relaxed);
            ISR_FLAG.store(false, Ordering::Relaxed);

            if count > 10 {
                neo.set_color(colors[color_index], 0.2)?;
                if color_index < colors.len() - 1 {
                    color_index += 1;
                } else {
                    color_index = 0;
                }
                count = 0;
            } else {
                count += 1;
            }
        } else {
            FreeRtos::delay_ms(1); // prevents WDT triggering
        }
    }
}
