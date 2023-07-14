#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use anyhow;
use csv;
use diff_drive::rigid2d::{Pose2D, Vector2D};
use diff_drive::utils;
use std::error::Error;
use std::f64::consts::PI;

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

/// computes the new pose of the robot by integrating forward
/// the dynamics. This will be replaced in real life with
/// the pose estimate from robot odometry
// fn simulate(pose: Pose2D<f64>, u1: f64, u2: f64, dt: f64) -> Pose2D<f64> {
//     let x_new = pose.x + f64::cos(pose.theta) * u1 * dt;
//     let y_new = pose.y + f64::sin(pose.theta) * u1 * dt;
//     let theta_new = pose.theta + u2 * dt;
//
//     Pose2D::new(x_new, y_new, theta_new)
// }

// const WHEEL_RADIUS: f32 = 0.033; // meters
const MAX_SPEED: f64 = 25.0;
fn main() -> anyhow::Result<()> {
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

    // for each pose in goal_pose_vec
    // - compute the optimal controls (left and right wheel speeds)
    // - get the pose of the robot at the next step

    Ok(())
}
