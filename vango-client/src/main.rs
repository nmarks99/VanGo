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
use diff_drive::rigid2d::Vector2D;

use vango_utils::{bytes_to_int, int_to_bytes};

const VANGO_SERVICE_ID: Uuid = Uuid::from_u128(0x21470560_232e_11ee_be56_0242ac120002);
const LEFT_SPEED_UUID: Uuid = Uuid::from_u128(0x3c9a3f00_8ed3_4bdf_8a39_a01bebede295);
const RIGHT_SPEED_UUID: Uuid = Uuid::from_u128(0xc0ffc89c_29bb_11ee_be56_0242ac120002);
const LEFT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a286b70_2c2b_11ee_be56_0242ac120002);
const RIGHT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a28672e_2c2b_11ee_be56_0242ac120002);
const WAYPOINT_UUID: Uuid = Uuid::from_u128(0x21e16dea_357a_11ee_be56_0242ac120002);
const PID_UUID: Uuid = Uuid::from_u128(0x3cedc40e_3655_11ee_be56_0242ac120002);

// const WHEEL_RADIUS: f64 = 0.042;
// const WHEEL_SEPARATION: f64 = 0.100;
// const MAX_RPM: u8 = 250;
const BASE_RPM: u8 = 100;

#[derive(PartialEq, Debug)]
enum Mode {
    Manual,
    Auto,
    Tune,
    Debug,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let mode = Mode::Tune;
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
    // info!("Discovered characteristics");

    let left_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == LEFT_SPEED_UUID)
        .ok_or("Left speed characteristic not found")?;

    let right_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == RIGHT_SPEED_UUID)
        .ok_or("Right speed characteristic not found")?;

    let left_counts_chr = characteristics
        .iter()
        .find(|x| x.uuid() == LEFT_COUNTS_UUID)
        .ok_or("Left count characteristic not found")?;

    let right_counts_chr = characteristics
        .iter()
        .find(|x| x.uuid() == RIGHT_COUNTS_UUID)
        .ok_or("Right count characteristic not found")?;

    let waypoint_chr = characteristics
        .iter()
        .find(|x| x.uuid() == WAYPOINT_UUID)
        .ok_or("Waypoint characteristic not found")?;

    let pid_chr = characteristics
        .iter()
        .find(|x| x.uuid() == PID_UUID)
        .ok_or("PID characteristic not found")?;

    info!("Connected all characteristics");

    // Manual mode is remote control mode with the keyboard
    if mode == Mode::Manual {
        let stdin = std::io::stdin();
        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        let mut speed: i16 = BASE_RPM.into();
        let mut left_speed: i16 = 0;
        let mut right_speed: i16 = 0;

        for c in stdin.keys() {
            write!(
                stdout,
                "{} {}",
                termion::clear::All,
                termion::cursor::Goto(1, 1)
            )
            .unwrap();

            // TODO: fix this so it properly "latches" and you can adjust speed while latched
            match c.unwrap() {
                // Forward
                Key::Char('i') => {
                    print!("Forward!\r\n");
                    left_speed = speed;
                    right_speed = speed;
                }

                // Backward
                Key::Char(',') => {
                    print!("Backward!\r\n");
                    left_speed = -speed;
                    right_speed = -speed;
                }

                // Clockwise
                Key::Char('l') => {
                    print!("Clockwise!\r\n");
                    left_speed = speed;
                    right_speed = -speed;
                }

                // Counter-clockwise
                Key::Char('j') => {
                    print!("Counter-Clockwise!\r\n");
                    left_speed = -speed;
                    right_speed = speed;
                }

                // Stop
                Key::Char('k') => {
                    print!("Stop!\r\n");
                    left_speed = 0;
                    right_speed = 0;
                }

                // Increase speed by 10%
                Key::Up => {
                    speed = speed + (speed as f32 * 0.1) as i16;
                }

                // Decrease speed by 10%
                Key::Down => {
                    speed = speed - (speed as f32 * 0.1) as i16;
                }

                // Exit
                Key::Char('q') => {
                    left_speed = 0;
                    right_speed = 0;
                    let right_speed_bytes = int_to_bytes(right_speed);
                    right_speed_chr
                        .write(&right_speed_bytes)
                        .await
                        .expect("failed to set right speed");

                    let left_speed_bytes = int_to_bytes(left_speed);
                    left_speed_chr
                        .write(&left_speed_bytes)
                        .await
                        .expect("failed to set right speed");

                    break;
                }
                _ => {}
            }

            let right_speed_bytes = int_to_bytes(right_speed);
            right_speed_chr
                .write(&right_speed_bytes)
                .await
                .expect("failed to set right speed");

            let left_speed_bytes = int_to_bytes(left_speed);
            left_speed_chr
                .write(&left_speed_bytes)
                .await
                .expect("failed to set right speed");

            stdout.flush().unwrap();
        }

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
        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        let right_speed_bytes: Vec<u8> = int_to_bytes(-50);
        right_speed_chr
            .write(&right_speed_bytes)
            .await
            .expect("failed to set right speed");

        let left_speed_bytes = int_to_bytes(-50);
        left_speed_chr
            .write(&left_speed_bytes)
            .await
            .expect("failed to set right speed");

        // Setup a task to print out info
        let left_counts_chr1 = left_counts_chr.clone();
        let right_counts_chr1 = right_counts_chr.clone();
        let left_speed_chr1 = left_speed_chr.clone();
        let right_speed_chr1 = right_speed_chr.clone();
        let task_handle = tokio::spawn(async move {
            loop {
                write!(
                    stdout,
                    "{}{}",
                    termion::cursor::Goto(1, 1),
                    termion::clear::CurrentLine
                )
                .unwrap();
                // all these reads are slow for some reason...
                let left_count_bytes = left_counts_chr1.read().await.unwrap();
                let right_count_bytes = right_counts_chr1.read().await.unwrap();
                let left_count_int = bytes_to_int::<i32>(&left_count_bytes).unwrap();
                let right_count_int = bytes_to_int::<i32>(&right_count_bytes).unwrap();

                let left_speed_bytes = left_speed_chr1.read().await.unwrap();
                let right_speed_bytes = right_speed_chr1.read().await.unwrap();
                let left_speed_int = bytes_to_int::<i32>(&left_speed_bytes).unwrap();
                let right_speed_int = bytes_to_int::<i32>(&right_speed_bytes).unwrap();

                print!("Count: {}, {}\r\n", left_count_int, right_count_int);
                print!("Speed: {}, {}\r\n", left_speed_int, right_speed_int);
            }
        });

        let stdin = std::io::stdin();
        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        write!(
            stdout,
            "{}{}q to exit{}",
            termion::clear::All,
            termion::cursor::Goto(1, 1),
            termion::cursor::Hide
        )
        .unwrap();
        stdout.flush().unwrap();
        for c in stdin.keys() {
            match c.unwrap() {
                Key::Char('q') => {
                    let right_speed_bytes: Vec<u8> = int_to_bytes(0);
                    right_speed_chr
                        .write(&right_speed_bytes)
                        .await
                        .expect("failed to set right speed");

                    let left_speed_bytes = int_to_bytes(0);
                    left_speed_chr
                        .write(&left_speed_bytes)
                        .await
                        .expect("failed to set right speed");
                    task_handle.abort();
                    break;
                }
                _ => {}
            }
            stdout.flush().unwrap();
        }
        write!(
            stdout,
            "{}{}{}",
            termion::clear::All,
            termion::cursor::Goto(1, 1),
            termion::cursor::Show
        )
        .unwrap();
        write!(stdout, "{}", termion::cursor::Show).unwrap();

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

        // let waypoint: Vector2D<f32> = Vector2D::new(2.71, 3.14);
        // let waypoint_vec = waypoint.to_vec();
        // let waypoint_string: Vec<String> = waypoint_vec.iter().map(|&f| f.to_string()).collect();
        // let waypoint_slice = &waypoint_string[..];
        // println!("waypoint string = {:?}", waypoint_slice);

        // waypoint_chr
        //     .write(&waypoint_slice)
        //     .await
        //     .expect("failed to set right speed");
    } else if mode == Mode::Tune {
        // loop {
        //
        //
        //
        //     pid_chr
        //         .write("P+1".as_bytes())
        //         .await
        //         .expect("Failed to write to PID characteristic");
        //     tokio::time::sleep(Duration::from_secs(1)).await;
        // }

        // Setup a task to generate a square wave
        // let left_speed_chr1 = left_speed_chr.clone();
        // let right_speed_chr1 = right_speed_chr.clone();
        // let task_handle = tokio::spawn(async move {
        //     loop {
        //         left_speed_chr1
        //             .write(&int_to_bytes(30))
        //             .await
        //             .expect("Failed to set left speed");
        //         right_speed_chr1
        //             .write(&int_to_bytes(30))
        //             .await
        //             .expect("Failed to set right speed");
        //         tokio::time::sleep(Duration::from_millis(500)).await;
        //         left_speed_chr1
        //             .write(&int_to_bytes(-30))
        //             .await
        //             .expect("Failed to set left speed");
        //         right_speed_chr1
        //             .write(&int_to_bytes(-30))
        //             .await
        //             .expect("Failed to set right speed");
        //         tokio::time::sleep(Duration::from_millis(500)).await;
        //     }
        // });

        left_speed_chr
            .write(&int_to_bytes(100))
            .await
            .expect("Failed to set left speed");
        right_speed_chr
            .write(&int_to_bytes(100))
            .await
            .expect("Failed to set right speed");
        let stdin = std::io::stdin();
        let mut stdout = std::io::stdout().into_raw_mode().unwrap();

        write!(
            stdout,
            "{}{}q to exit{}",
            termion::clear::All,
            termion::cursor::Goto(1, 1),
            termion::cursor::Hide
        )
        .unwrap();
        stdout.flush().unwrap();
        for c in stdin.keys() {
            match c.unwrap() {
                Key::Char('q') => {
                    right_speed_chr
                        .write(&int_to_bytes(0))
                        .await
                        .expect("failed to set right speed");
                    left_speed_chr
                        .write(&int_to_bytes(0))
                        .await
                        .expect("failed to set right speed");
                    // task_handle.abort();
                    break;
                }

                Key::Char('P') => {
                    pid_chr
                        .write("P+2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                Key::Char('p') => {
                    pid_chr
                        .write("P-2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                Key::Char('I') => {
                    pid_chr
                        .write("I+2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                Key::Char('i') => {
                    pid_chr
                        .write("I-2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                Key::Char('D') => {
                    pid_chr
                        .write("D+2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                Key::Char('d') => {
                    pid_chr
                        .write("D-2".as_bytes())
                        .await
                        .expect("Failed to write to PID characteristic");
                }

                _ => {}
            }
            stdout.flush().unwrap();
        }
        write!(
            stdout,
            "{}{}{}",
            termion::clear::All,
            termion::cursor::Goto(1, 1),
            termion::cursor::Show
        )
        .unwrap();
        write!(stdout, "{}", termion::cursor::Show).unwrap();
    }

    Ok(())
}
