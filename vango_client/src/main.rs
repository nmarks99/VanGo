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

const NEOPIXEL_CHARACTERISTIC: Uuid = Uuid::from_u128(0x3c9a3f00_8ed3_4bdf_8a39_a01bebede295);
const VANGO_SERVICE_ID: Uuid = Uuid::from_u128(0x21470560_232e_11ee_be56_0242ac120002);

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

const WHEEL_RADIUS: f64 = 0.042;
const WHEEL_SEPARATION: f64 = 0.100;
const MAX_SPEED: f64 = 25.0;

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

    // let mut neopixel_chr: Option<&Characteristic> = None;
    let neopixel_chr = characteristics
        .iter()
        .find(|x| x.uuid() == NEOPIXEL_CHARACTERISTIC)
        .ok_or("Neopixel characteristic not found")?;
    neopixel_chr.write(&[0x30]).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    neopixel_chr.write(&[0x31]).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    neopixel_chr.write(&[0x33]).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    neopixel_chr.write(&[0x34]).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    neopixel_chr.write(&[0x35]).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    let stdin = std::io::stdin();
    let mut stdout = std::io::stdout().into_raw_mode().unwrap();

    // clear the screen, move and hide cursor
    // write!(
    //     stdout,
    //     "{} {} {}",
    //     termion::clear::All,
    //     termion::cursor::Goto(1, 1),
    //     termion::cursor::Hide
    // )
    // .unwrap();
    //
    // stdout.flush().unwrap();

    // ==============================

    // assume the robot start at 0,0,pi/2
    let start: Pose2D<f64> = Pose2D::new(0.0, 0.0, PI / 2.0);

    // Get points along target trajectory
    let points: Vec<Vector2D<f64>> = read_csv_trajectory("traj.csv")?;

    // compute goal angle for each point along trajectory
    // note that we are normalizing between -pi and pi
    let mut theta_vec: Vec<f64> = vec![start.theta];
    for i in 0..points.len() - 1 {
        let th = utils::normalize_angle(
            (points[i + 1].y - points[i].y).atan2(points[i + 1].x - points[i].x),
        );
        theta_vec.push(th);
    }

    let mut goal_pose_vec: Vec<Pose2D<f64>> = vec![start];
    for i in 0..points.len() {
        let pose = Pose2D::new(points[i].x, points[i].y, theta_vec[i]);
        goal_pose_vec.push(pose);
    }

    for p in &goal_pose_vec {
        info!("{} {} {}", p.x, p.y, p.theta)
    }

    // for each pose in goal_pose_vec:
    // - get the wheel angles from encoders with BLE
    // - get current pose estimate from odometry given wheel angles (FK)
    // - compute the optimal controls (wheel speeds)
    // - send control signals to motors with BLE

    Ok(())
}
