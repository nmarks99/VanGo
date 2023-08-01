#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use anyhow;
use csv;
use diff_drive::rigid2d::{Pose2D, Vector2D};
use diff_drive::utils;
use std::error::Error;
use std::f64::consts::PI;

use bluest::{Adapter, Characteristic, Uuid};
use futures_util::StreamExt;
use std::time::Duration;
use tracing::info;
use tracing::metadata::LevelFilter;

use std::io::Write;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

use num_traits::{PrimInt, Signed};

mod cluster;

const VANGO_SERVICE_ID: Uuid = Uuid::from_u128(0x21470560_232e_11ee_be56_0242ac120002);

const LEFT_SPEED_UUID: Uuid = Uuid::from_u128(0x3c9a3f00_8ed3_4bdf_8a39_a01bebede295);
const RIGHT_SPEED_UUID: Uuid = Uuid::from_u128(0xc0ffc89c_29bb_11ee_be56_0242ac120002);

const LEFT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a286b70_2c2b_11ee_be56_0242ac120002);
const RIGHT_COUNTS_UUID: Uuid = Uuid::from_u128(0x0a28672e_2c2b_11ee_be56_0242ac120002);

fn linspace(start: f32, stop: f32, num_points: usize) -> Vec<f32> {
    let step = (stop - start) / (num_points - 1) as f32;
    (0..num_points).map(|i| start + (i as f32) * step).collect()
}

fn create_ref_traj() -> anyhow::Result<(Vec<f32>, Vec<f32>)> {
    let mut traj_file = csv::Writer::from_path("traj.csv")?;

    // create a semi-circle trajectory
    const RADIUS: f32 = 2.0;
    let traj_x = linspace(0.0, RADIUS * 2.0, 100);
    let mut traj_y: Vec<f32> = vec![];
    for xi in &traj_x {
        let y = (RADIUS.powf(2.0) - (*xi as f32 - RADIUS).powf(2.0)).sqrt();
        traj_y.push(y);
    }

    for i in 0..traj_x.len() {
        let line = [traj_x[i], traj_y[i]].map(|e| e.to_string());
        traj_file.write_record(line)?;
    }

    traj_file.flush()?;
    Ok((traj_x, traj_y))
}

fn read_csv_trajectory(csv_path: &str) -> anyhow::Result<Vec<Vector2D<f64>>> {
    let mut reader = csv::Reader::from_path(csv_path)?;

    let mut points: Vec<Vector2D<f64>> = vec![];

    // note this skips the first line of the csv file
    for res in reader.records() {
        let record = res?;
        let x: f64 = record[0].parse().unwrap();
        let y: f64 = record[1].parse().unwrap();
        let v = Vector2D::new(x, y);
        points.push(v);
    }

    Ok(points)
}

// Converts a signed integer to a Vec<u8>
pub fn int_to_bytes<T: PrimInt + Signed>(num: T) -> Vec<u8> {
    let num_string = (num.to_i64().unwrap()).to_string();
    let num_bytes_vec = num_string.as_bytes().to_vec();
    num_bytes_vec
}

const WHEEL_RADIUS: f64 = 0.042;
const WHEEL_SEPARATION: f64 = 0.100;
// const MAX_SPEED: f64 = 25.0;
const MAX_RPM: u8 = 250;
const BASE_RPM: u8 = 100;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
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

    let adapter = Adapter::default()
        .await
        .ok_or("Bluetooth adapter not found")?;
    adapter.wait_available().await?;

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
    info!("Connected!");

    let service = match device
        .discover_services_with_uuid(VANGO_SERVICE_ID)
        .await?
        .get(0)
    {
        Some(service) => service.clone(),
        None => return Err("Service not found".into()),
    };
    info!("Vango service connected");

    let characteristics = service.discover_characteristics().await?;
    info!("Discovered characteristics");

    let left_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == LEFT_SPEED_UUID)
        .ok_or("Left speed characteristic not found")?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    let right_speed_chr = characteristics
        .iter()
        .find(|x| x.uuid() == RIGHT_SPEED_UUID)
        .ok_or("Right speed characteristic not found")?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    let left_counts_chr = characteristics
        .iter()
        .find(|x| x.uuid() == LEFT_COUNTS_UUID)
        .ok_or("Left count characteristic not found")?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    let right_counts_chr = characteristics
        .iter()
        .find(|x| x.uuid() == RIGHT_COUNTS_UUID)
        .ok_or("Right count characteristic not found")?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    let stdin = std::io::stdin();
    let mut stdout = std::io::stdout().into_raw_mode().unwrap();
    let mut speed: i16 = BASE_RPM.into();
    let mut left_speed: i16 = -50;
    let mut right_speed: i16 = 50;

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
                println!("Forward!");
                // equal positive left and right speeds
                left_speed = speed;
                right_speed = speed;
            }

            // Backward
            Key::Char(',') => {
                println!("Backward!");
                left_speed = -speed;
                right_speed = -speed;
                // equal negative left and right speeds
            }

            // Clockwise
            Key::Char('l') => {
                println!("Clockwise!");
                left_speed = speed;
                right_speed = -speed;
                // opposite direction, equal magnitude left and right speeds
            }

            // Counter-clockwise
            Key::Char('j') => {
                println!("Counter-Clockwise!");
                left_speed = -speed;
                right_speed = speed;
                // opposite direction, equal magnitude left and right speeds
            }

            // Stop
            Key::Char('k') => {
                // both left and right speeds are zero
                println!("Stop!");
                left_speed = 0;
                right_speed = 0;
            }

            // Increase speed
            Key::Up => {
                speed = speed + (speed as f32 * 0.1) as i16;
            }

            // Decrease speed
            Key::Down => {
                speed = speed - (speed as f32 * 0.1) as i16;
            }

            // Exit
            Key::Char('q') => break,
            _ => {}
        };

        // TODO: this won't work, need to handle signs to convert i16 to &[u8]
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

        let left_count = left_counts_chr.read().await.expect("read failed");
        let right_count = right_counts_chr.read().await.expect("read failed");
        // println!("Left count = {:?}", left_count);
        // println!("Left count = {:?}", left_count);
        // if cmd.is_some() {
        //     if cmd.unwrap() == "LEFT" {
        //         let left_speed = left_speed_chr.read().await.expect("Read failed");
        //         print!("Left speed = {:?}", left_speed[0]);
        //     } else if cmd.unwrap() == "RIGHT" {
        //         let right_speed = right_speed_chr.read().await.expect("Read failed");
        //         print!("Right speed = {:?}", right_speed[0]);
        //     } else if cmd.unwrap() == "START" {
        //         print!("Setting right speed to 100");
        //         let speed_str = "100";
        //         right_speed_chr
        //             .write(speed_str.as_bytes())
        //             .await
        //             .expect("Failed to write");
        //     } else if cmd.unwrap() == "STOP" {
        //         print!("Stopping right motor");
        //         let speed_str = "0";
        //         right_speed_chr
        //             .write(speed_str.as_bytes())
        //             .await
        //             .expect("Failed to write");
        //     }
        // }

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

    // ==============================

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

    Ok(())
}
