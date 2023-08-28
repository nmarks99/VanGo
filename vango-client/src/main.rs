#![allow(dead_code)]
use std::error::Error;

use bluest::{Adapter, Uuid};
use futures_util::StreamExt;
// use std::time::Duration;
use tracing::info;
use tracing::metadata::LevelFilter;

use std::io::Write;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

mod cluster;
use diff_drive::ddrive::{DiffDrive, WheelState};
use diff_drive::rigid2d::{Pose2D, Twist2D, Vector2D};
use diff_drive::trajectory::Path;
use diff_drive::utils::{normalize_angle, rad2deg};
use vango_utils as utils;
use vango_utils::{almost_equal, ascii_to_f32, f32_to_ascii, get_min_angle};

const VANGO_SERVICE_ID: Uuid = Uuid::from_u128(0x21470560_232e_11ee_be56_0242ac120002);
const LEFT_SPEED_UUID: Uuid = Uuid::from_u128(0x3c9a3f00_8ed3_4bdf_8a39_a01bebede295);
const RIGHT_SPEED_UUID: Uuid = Uuid::from_u128(0xc0ffc89c_29bb_11ee_be56_0242ac120002);
const LEFT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a286b70_2c2b_11ee_be56_0242ac120002);
const RIGHT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a28672e_2c2b_11ee_be56_0242ac120002);
const WAYPOINT_UUID: Uuid = Uuid::from_u128(0x21e16dea_357a_11ee_be56_0242ac120002);
const POSE_THETA_UUID: Uuid = Uuid::from_u128(0x3cedc40e_3655_11ee_be56_0242ac120002);
const POSE_X_UUID: Uuid = Uuid::from_u128(0xa0c2b3b2_3b1a_11ee_be56_0242ac120002);
const POSE_Y_UUID: Uuid = Uuid::from_u128(0xa0c2b65a_3b1a_11ee_be56_0242ac120002);
const PEN_UUID: Uuid = Uuid::from_u128(0x0daaac7c_3d6a_11ee_be56_0242ac120002);

const BASE_SPEED: f32 = 1.5;
const WHEEL_RADIUS: f32 = 0.045 / 2.0; // meters
const WHEEL_SEPARATION: f32 = 0.105; // meters
const BYTES_ZERO: &[u8; 1] = &[b'0'];
const BYTES_ONE: &[u8; 1] = &[b'1'];

#[derive(PartialEq, Debug)]
enum Mode {
    Manual,
    Auto,
    Debug,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let mode = Mode::Manual;
    println!("Mode: {:?}", mode);

    // TODO: use clap for command line args
    // let args: Vec<String> = env::args().collect();
    // for i in &args {
    //     println!("{i}");
    // }

    // not sure what this part does
    use tracing_subscriber::prelude::*;
    use tracing_subscriber::{fmt, EnvFilter};
    tracing_subscriber::registry()
        .with(fmt::layer())
        .with(
            EnvFilter::builder()
                .with_default_directive(LevelFilter::INFO.into())
                .from_env_lossy(),
        )
        .init();

    // instatiate BLE adapter
    info!("Looking for bluetooth adapter...");
    let adapter = Adapter::default()
        .await
        .ok_or("Bluetooth adapter not found")?;
    adapter.wait_available().await?;

    // find the BLE device service (robot) from the UUID
    let device = adapter
        .discover_devices(&[VANGO_SERVICE_ID])
        .await?
        .next()
        .await
        .ok_or("Failed to discover device")??;
    info!(
        "Found device: {} ({:?})",
        device.name().as_deref().unwrap_or("(unknown)"),
        device.id()
    );
    adapter.connect_device(&device).await?;

    let service = match device
        .discover_services_with_uuid(VANGO_SERVICE_ID)
        .await?
        .get(0)
    {
        Some(service) => service.clone(),
        None => return Err("Service not found".into()),
    };
    info!("Vango service connected");

    // Find and connect to all the characteristics
    let characteristics = service.discover_characteristics().await?;

    let left_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == LEFT_SPEED_UUID)
        .ok_or("Left speed characteristic not found")?;

    let right_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == RIGHT_SPEED_UUID)
        .ok_or("Right speed characteristic not found")?;

    let pose_x_chr = characteristics
        .iter()
        .find(|x| x.uuid() == POSE_X_UUID)
        .ok_or("Pose x characteristic not found")?;

    let pose_y_chr = characteristics
        .iter()
        .find(|x| x.uuid() == POSE_Y_UUID)
        .ok_or("Pose y characteristic not found")?;

    let pose_theta_chr = characteristics
        .iter()
        .find(|x| x.uuid() == POSE_THETA_UUID)
        .ok_or("Pose theta characteristic not found")?;

    let pen_chr = characteristics
        .iter()
        .find(|x| x.uuid() == PEN_UUID)
        .ok_or("Pen characteristic not found")?;

    info!("Connected all characteristics");

    // Manual mode is remote control mode with the keyboard
    if mode == Mode::Manual {
        let stdin = std::io::stdin();
        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        let mut speed: f32 = BASE_SPEED.into();
        let mut left_speed: f32 = 0.0;
        let mut right_speed: f32 = 0.0;
        let pose_x_chr1 = pose_x_chr.clone();
        let pose_y_chr1 = pose_y_chr.clone();
        let pose_theta_chr1 = pose_theta_chr.clone();
        let task_handle = tokio::spawn(async move {
            write!(
                stdout,
                "{}{}",
                termion::clear::All,
                termion::cursor::Goto(1, 1),
            )
            .unwrap();
            loop {
                write!(
                    stdout,
                    "{}{}",
                    termion::cursor::Goto(1, 1),
                    termion::clear::CurrentLine
                )
                .unwrap();

                let pose = Pose2D::new(
                    ascii_to_f32(pose_x_chr1.read().await.unwrap()).unwrap(),
                    ascii_to_f32(pose_y_chr1.read().await.unwrap()).unwrap(),
                    ascii_to_f32(pose_theta_chr1.read().await.unwrap()).unwrap(),
                );
                print!("{}\n", pose);

                // tokio::time::sleep(Duration::from_millis(100)).await;
            }
        });

        for c in stdin.keys() {
            match c.unwrap() {
                // Forward
                Key::Char('i') => {
                    // print!("Forward!\r\n");
                    left_speed = speed;
                    right_speed = speed;
                }

                // Backward
                Key::Char(',') => {
                    // print!("Backward!\r\n");
                    left_speed = -speed;
                    right_speed = -speed;
                }

                // Clockwise
                Key::Char('l') => {
                    // print!("Clockwise!\r\n");
                    left_speed = speed;
                    right_speed = -speed;
                }

                // Counter-clockwise
                Key::Char('j') => {
                    // print!("Counter-Clockwise!\r\n");
                    left_speed = -speed;
                    right_speed = speed;
                }

                // Stop
                Key::Char('k') => {
                    // print!("Stop!\r\n");
                    left_speed = 0.0;
                    right_speed = 0.0;
                }

                Key::Char('u') => {
                    pen_chr.write(BYTES_ONE).await.unwrap();
                }

                Key::Char('n') => {
                    pen_chr.write(BYTES_ZERO).await.unwrap();
                }

                Key::Char('z') => {
                    info!("Zero!");
                    pose_x_chr.write(BYTES_ZERO).await.unwrap();
                    pose_y_chr.write(BYTES_ZERO).await.unwrap();
                    pose_theta_chr.write(BYTES_ZERO).await.unwrap();
                }

                // Increase speed by 10%
                Key::Up => {
                    speed = speed + (speed as f32 * 0.1);
                }

                // Decrease speed by 10%
                Key::Down => {
                    speed = speed - (speed as f32 * 0.1);
                }

                // Exit
                Key::Char('q') => {
                    left_speed = 0.0;
                    right_speed = 0.0;
                    let right_speed_bytes = f32_to_ascii(right_speed);
                    right_speed_chr
                        .write(&right_speed_bytes)
                        .await
                        .expect("failed to set right speed");

                    let left_speed_bytes = f32_to_ascii(left_speed);
                    left_speed_chr
                        .write(&left_speed_bytes)
                        .await
                        .expect("failed to set right speed");
                    task_handle.abort();
                    break;
                }
                _ => {}
            }

            let right_speed_bytes = f32_to_ascii(right_speed);
            right_speed_chr
                .write(&right_speed_bytes)
                .await
                .expect("failed to set right speed");

            let left_speed_bytes = f32_to_ascii(left_speed);
            left_speed_chr
                .write(&left_speed_bytes)
                .await
                .expect("failed to set right speed");

            // stdout.flush().unwrap();
        }

        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        // clear screen, move and show cursor at the end
        write!(
            stdout,
            "{} {} {}",
            termion::clear::All,
            termion::cursor::Goto(1, 1),
            termion::cursor::Show
        )
        .unwrap();

    // Autonomous mode
    } else if mode == Mode::Auto {
        // let path = Path::semi_circle(0.2, 50);
        // let path_vec = path.to_vec();
        let path_vec = vec![
            // Vector2D::new(0.0, 0.3),
            Vector2D::new(0.2, 0.2),
            Vector2D::new(0.2, 0.0),
            Vector2D::new(0.0, 0.0),
        ];

        const KP: f32 = 1.4;
        const KI: f32 = 0.0;
        const KD: f32 = 0.1;
        const THESHOLD_DISTANCE: f32 = 0.01;
        let mut controller = PidController::new(KP, KI, KD);
        let robot = DiffDrive::new(WHEEL_RADIUS, WHEEL_SEPARATION);
        let mut count = 1;

        pen_chr.write(BYTES_ZERO).await.unwrap();
        for target_point in path_vec {
            loop {
                // get current pose of the robot
                let pose = Pose2D::new(
                    ascii_to_f32(pose_x_chr.read().await.expect("failed to read pose.x"))
                        .expect("failed to convert pose.x ascii to f32"),
                    ascii_to_f32(pose_y_chr.read().await.expect("failed to read pose.y"))
                        .expect("failed to convert pose.y ascii to f32"),
                    ascii_to_f32(
                        pose_theta_chr
                            .read()
                            .await
                            .expect("failed to read pose.theta"),
                    )
                    .expect("failed to convert pose.theta ascii to f32"),
                );
                let point = Vector2D::new(pose.x, pose.y);
                println!("{},{}", point.x, point.y);
                let dist = point.distance(target_point);
                if dist <= THESHOLD_DISTANCE {
                    break;
                }

                let target_angle = normalize_angle(f32::atan2(
                    target_point.y - point.y,
                    target_point.x - point.x,
                ));

                // Compute angular speed from controller
                let err = get_min_angle(target_angle, pose.theta);
                let u = controller.compute(err); // u is thetadot
                let speeds = robot.speeds_from_twist(Twist2D::new(u, 0.05, 0.0));

                // Set wheel speeds
                let left_speed_bytes = f32_to_ascii(speeds.left);
                left_speed_chr
                    .write(&left_speed_bytes)
                    .await
                    .expect("set left speed failed");
                let right_speed_bytes = f32_to_ascii(speeds.right);
                right_speed_chr
                    .write(&right_speed_bytes)
                    .await
                    .expect("set right speed failed");
            }
            count += 1;
        }
        info!("Done!");
        pen_chr.write(BYTES_ONE).await.unwrap();
        left_speed_chr
            .write(&[b'0'])
            .await
            .expect("failed to set left zero");
        right_speed_chr
            .write(&[b'0'])
            .await
            .expect("failed to set right zero");
    } else if mode == Mode::Debug {
        println!("Debug mode unimplemented");
    }

    Ok(())
}

struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    err_prev: f32,
    i_err: f32,
}

impl PidController {
    fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            err_prev: 0.0,
            i_err: 0.0,
        }
    }

    fn compute(&mut self, error: f32) -> f32 {
        self.i_err += error;
        let d_err: f32 = error - self.err_prev;
        self.err_prev = error;
        self.kp * error + self.ki * self.i_err + self.ki * d_err
    }
}
