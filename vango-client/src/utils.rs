#![allow(dead_code)]

use anyhow;
use csv;
use diff_drive::rigid2d::Vector2D;
use num_traits::{PrimInt, Signed};

pub fn linspace(start: f32, stop: f32, num_points: usize) -> Vec<f32> {
    let step = (stop - start) / (num_points - 1) as f32;
    (0..num_points).map(|i| start + (i as f32) * step).collect()
}

pub fn create_ref_traj() -> anyhow::Result<(Vec<f32>, Vec<f32>)> {
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

pub fn read_csv_trajectory(csv_path: &str) -> anyhow::Result<Vec<Vector2D<f64>>> {
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

/// Converts an array of ASCII byte literals to a signed integer
/// Example:
/// let x1: i16 = bytes_to_int(&[b'-', b'1', b'2']).unwrap();
/// let x2: i16 = bytes_to_int(&[b'1', b'2']).unwrap();
/// x1 here will be -12, x2 will be 12
pub fn bytes_to_int<T: PrimInt + Signed>(arr: &[u8]) -> Option<T> {
    let mut result = T::zero();
    let mut sign: i8 = 1;
    for i in 0..arr.len() {
        if i == 0 {
            if arr[i] == b'-' {
                sign = -1;
                continue;
            }
        }
        if arr[i] >= b'0' && arr[i] <= b'9' {
            result =
                result * T::from(10).unwrap() + (T::from(arr[i]).unwrap() - T::from(b'0').unwrap());
        } else {
            return None;
        }
    }
    Some(T::from(sign).unwrap() * result)
}

pub fn u8_to_bytes(arr: &[u8]) -> Vec<u8> {
    let mut byte_literals = Vec::with_capacity(arr.len());
    for &digit in arr {
        // Convert the u8 digit to its corresponding ASCII code
        let ascii_code = b'0' + digit;
        byte_literals.push(ascii_code);
    }
    byte_literals
}
