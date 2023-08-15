#![allow(dead_code)]
use std::error::Error;

use bluest::{Adapter, Uuid};
use futures_util::StreamExt;
use std::time::Duration;
use tracing::info;
use tracing::metadata::LevelFilter;

use std::io::Write;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

mod cluster;
use diff_drive::rigid2d::Pose2D;
use vango_utils::{ascii_to_f32, f32_to_ascii};

const VANGO_SERVICE_ID: Uuid = Uuid::from_u128(0x21470560_232e_11ee_be56_0242ac120002);
const LEFT_SPEED_UUID: Uuid = Uuid::from_u128(0x3c9a3f00_8ed3_4bdf_8a39_a01bebede295);
const RIGHT_SPEED_UUID: Uuid = Uuid::from_u128(0xc0ffc89c_29bb_11ee_be56_0242ac120002);
const LEFT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a286b70_2c2b_11ee_be56_0242ac120002);
const RIGHT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a28672e_2c2b_11ee_be56_0242ac120002);
const WAYPOINT_UUID: Uuid = Uuid::from_u128(0x21e16dea_357a_11ee_be56_0242ac120002);
const POSE_THETA_UUID: Uuid = Uuid::from_u128(0x3cedc40e_3655_11ee_be56_0242ac120002);
const POSE_X_UUID: Uuid = Uuid::from_u128(0xa0c2b3b2_3b1a_11ee_be56_0242ac120002);
const POSE_Y_UUID: Uuid = Uuid::from_u128(0xa0c2b65a_3b1a_11ee_be56_0242ac120002);

// const WHEEL_RADIUS: f64 = 0.042;
// const WHEEL_SEPARATION: f64 = 0.100;
// const MAX_RPM: u8 = 250;
const BASE_SPEED: f32 = 10.0;

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

    // let left_counts_chr = characteristics
    //     .iter()
    //     .find(|x| x.uuid() == LEFT_COUNTS_UUID)
    //     .ok_or("Left count characteristic not found")?;
    //
    // let right_counts_chr = characteristics
    //     .iter()
    //     .find(|x| x.uuid() == RIGHT_COUNTS_UUID)
    //     .ok_or("Right count characteristic not found")?;

    let waypoint_chr = characteristics
        .iter()
        .find(|x| x.uuid() == WAYPOINT_UUID)
        .ok_or("Waypoint characteristic not found")?;

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
            // write!(
            //     stdout,
            //     "{} {}",
            //     termion::clear::All,
            //     termion::cursor::Goto(1, 1)
            // )
            // .unwrap();

            // TODO: fix this so it properly "latches" and you can adjust speed while latched
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
        println!("Auto mode unimplemented");

        // =============================

        // assume the robot start at 0,0,pi/2
        // let start: Pose2D<f64> = Pose2D::new(0.0, 0.0, PI / 2.0);

        // Get points along target trajectory
        // let points: Vec<Vector2D<f64>> = read_csv_trajectory("traj.csv")?;

        // compute goal angle for each point along trajectory
        // note that we are normalizing between -pi and pi
        // let mut theta_vec: Vec<f64> = vec![start.theta];
        // for i in 0..points.len() - 1 {
        //     let th = utils::normalize_angle(
        //         (points[i + 1].y - points[i].y).atan2(points[i + 1].x - points[i].x),
        //     );
        //     theta_vec.push(th);
        // }
        //
        // let mut goal_pose_vec: Vec<Pose2D<f64>> = vec![start];
        // for i in 0..points.len() {
        //     let pose = Pose2D::new(points[i].x, points[i].y, theta_vec[i]);
        //     goal_pose_vec.push(pose);
        // }

        // for each pose in goal_pose_vec:
        // - get the wheel angles from encoders with BLE
        // - get current pose estimate from odometry given wheel angles (FK)
        // - compute the optimal controls (wheel speeds)
        // - send control signals to motors with BLE
    } else if mode == Mode::Debug {
        println!("Debug mode unimplemented");
    }

    Ok(())
}
