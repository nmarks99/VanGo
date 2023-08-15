// basics
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
mod motor;
mod neopixel;
mod pen;
use encoder::Encoder;
use encoder::{ENCODER_RATE_MS, TICKS_PER_RAD};
use motor::MotorDirection;
use neopixel::Neopixel;
use vango_utils as utils;
// use pen::{Pen, PenState};

// Atomic variables set in timer ISR
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

// BLE UUIDs
const VANGO_SERVICE_UUID: BleUuid = uuid128!("21470560-232e-11ee-be56-0242ac120002");
const LEFT_SPEED_UUID: BleUuid = uuid128!("3c9a3f00-8ed3-4bdf-8a39-a01bebede295");
const RIGHT_SPEED_UUID: BleUuid = uuid128!("c0ffc89c-29bb-11ee-be56-0242ac120002");
const LEFT_COUNTS_UUID: BleUuid = uuid128!("0a286b70-2c2b-11ee-be56-0242ac120002");
const RIGHT_COUNTS_UUID: BleUuid = uuid128!("0a28672e-2c2b-11ee-be56-0242ac120002");
const WAYPOINT_UUID: BleUuid = uuid128!("21e16dea-357a-11ee-be56-0242ac120002");
// const PID_UUID: BleUuid = uuid128!("3cedc40e-3655-11ee-be56-0242ac120002");

// PD controller gains
const KP: f32 = 0.3;
const KD: f32 = 1.2;
const DIR_CHANGE_THRESHOLD: f32 = 0.5;

// Robot paramaters
const WHEEL_RADIUS: f32 = 0.045 / 2.0; // meters
const WHEEL_SEPARATION: f32 = 0.103; // meters

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    esp_idf_svc::log::EspLogger::initialize_default();

    // let mut waypoints_vec: Vec<>

    // Set up neopixel
    let mut neo = Neopixel::new(peripherals.pins.gpio21, peripherals.rmt.channel0)?;
    neo.set_color("white", 0.2)?;

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

    // BLE characteristic for left motor speed
    let left_speed_blec = ble_service.lock().create_characteristic(
        LEFT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    left_speed_blec
        .lock()
        .on_read(move |v, _| {
            // let speed = LEFT_RPM.load(Ordering::Relaxed);
            let speed = LEFT_SPEED.load(Ordering::Relaxed);
            // v.set_value(&utils::int_to_bytes(speed));
            v.set_value(&utils::int_to_bytes(0));
            log::info!("Left speed = {:?}", speed);
        })
        .on_write(move |recv| {
            let rpm_target_recv: i16 = utils::bytes_to_int(recv.recv_data).unwrap();
            log::info!("Left speed target {:?}", rpm_target_recv);
            TARGET_SPEED_LEFT.store(rpm_target_recv as f32, Ordering::SeqCst);
        });

    // BLE characteristic for right motor speed
    let right_speed_blec = ble_service.lock().create_characteristic(
        RIGHT_SPEED_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE,
    );
    right_speed_blec
        .lock()
        .on_read(move |v, _| {
            // let speed = RIGHT_RPM.load(Ordering::Relaxed);
            let speed = RIGHT_SPEED.load(Ordering::Relaxed);
            // v.set_value(&utils::int_to_bytes(speed));
            v.set_value(&utils::int_to_bytes(0));
            log::info!("Right speed = {:?}", speed);
        })
        .on_write(move |recv| {
            let rpm_target_recv: i16 = utils::bytes_to_int(recv.recv_data).unwrap();
            log::info!("Right speed target {:?}", rpm_target_recv);
            TARGET_SPEED_RIGHT.store(rpm_target_recv as f32, Ordering::SeqCst);
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

    // Set initial motor direction, both forward
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

    const ALPHA: f32 = 0.1;
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

            // Handle target speeds for both directions.
            let mut left_motor_dir: MotorDirection;
            let mut right_motor_dir: MotorDirection;
            if target_speed_left < 0.0 {
                // Backward
                u_left = -u_left;
                let _ = left_direction.set_high();
            } else {
                // Forward
                let _ = left_direction.set_low();
            }
            if target_speed_right < 0.0 {
                // Backward
                u_right = -u_right;
                let _ = right_direction.set_low();
            } else {
                // Forward
                let _ = right_direction.set_high();
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
            let _ = left_pwm_driver.set_duty(left_duty as u32);
            let _ = right_pwm_driver.set_duty(right_duty as u32);

            println!("{},{}", left_speed, right_speed);
        })
        .unwrap();

    task_timer
        .every(Duration::from_millis(ENCODER_RATE_MS))
        .unwrap();

    let mut robot = DiffDrive::new(WHEEL_RADIUS, WHEEL_SEPARATION);
    loop {
        let target_twist = Twist2D::new(0.0, 0.2, 0.0);
        let target_speeds = robot.speeds_from_twist(target_twist);
        TARGET_SPEED_LEFT.store(target_speeds.left, Ordering::Relaxed);
        TARGET_SPEED_RIGHT.store(target_speeds.right, Ordering::Relaxed);
        // println!("Speeds = {}", target_speeds);
        FreeRtos::delay_ms(3000);

        let target_twist = Twist2D::new(0.0, -0.2, 0.0);
        let target_speeds = robot.speeds_from_twist(target_twist);
        TARGET_SPEED_LEFT.store(target_speeds.left, Ordering::Relaxed);
        TARGET_SPEED_RIGHT.store(target_speeds.right, Ordering::Relaxed);
        // println!("Speeds = {}", target_speeds);
        FreeRtos::delay_ms(3000);
    }

    // let mut count = 0;
    // let mut robot = DiffDrive::new(WHEEL_RADIUS, WHEEL_SEPARATION);
    // let mut t0 = SYS_TIMER.now().as_millis();
    // let target_twist = Twist2D::new(0.0, 0.2, 0.0);
    // let target_speeds = robot.speeds_from_twist(target_twist);
    // TARGET_SPEED_LEFT.store(target_speeds.left, Ordering::Relaxed);
    // TARGET_SPEED_RIGHT.store(target_speeds.right, Ordering::Relaxed);
    // loop {
    //     let isr_flag = ISR_FLAG.load(Ordering::Relaxed);
    //     let t = SYS_TIMER.now().as_millis() - t0;
    //     if isr_flag {
    //         let wheel_angles = WheelState::new(
    //             LEFT_ANGLE.load(Ordering::Relaxed),
    //             RIGHT_ANGLE.load(Ordering::Relaxed),
    //         );
    //         let pose = robot.forward_kinematics(wheel_angles);
    //         if count >= 1 {
    //             println!(
    //                 "{},{},{},{},{},{}",
    //                 t, wheel_angles.left, wheel_angles.right, pose.theta, pose.x, pose.y
    //             );
    //             count = 0;
    //         } else {
    //             count += 1;
    //         }
    //         ISR_FLAG.store(false, Ordering::Relaxed);
    //     } else {
    //         FreeRtos::delay_ms(1); // prevents WDT triggering
    //     }
    // }
}
