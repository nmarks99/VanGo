//! [![github]](https://github.com/nmarks99/VanGo/vango-utils)&ensp;
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//!
//! <br>
//!
//! This library contains various functions used by both the VanGo
//! firmware, and VanGo desktop client.
//!
//! <br>
//!
//! # Details
//! TODO: explain more here

use log::warn;
use num_traits::{Float, PrimInt, Signed};

/// checks if two floats are within some threshold of each of other
pub fn almost_equal<T: Float>(d1: T, d2: T, epsilon: T) -> bool {
    (d1 - d2).abs() < epsilon
}

/// linear mapping between two ranges, similar to arduino's map function
pub fn map<T: PrimInt>(x: T, xmin: T, xmax: T, ymin: T, ymax: T) -> T {
    assert!(xmin < xmax, "xmin must be less than xmax");
    let x_ratio = (x - xmin).to_f32().unwrap() / (xmax - xmin).to_f32().unwrap();
    let res = ymin.to_f32().unwrap() + x_ratio * (ymax - ymin).to_f32().unwrap();
    T::from(res).expect("conversion failed")
}

/// Converts a slice of byte literals to a signed integer
/// Example:
/// ```
/// use vango_utils::bytes_to_int;
/// let x1: i16 = bytes_to_int(&[b'-', b'1', b'2']).unwrap();
/// let x2: i16 = bytes_to_int(&[b'1', b'2']).unwrap();
/// ```
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

/// Converts a signed integer to a Vec<u8>
pub fn int_to_bytes<T: PrimInt + Signed>(num: T) -> Vec<u8> {
    let num_string: String;
    if cfg!(target_pointer_width = "64") {
        num_string = (num.to_i64().unwrap()).to_string();
    } else if cfg!(target_pointer_width = "32") {
        num_string = (num.to_i32().unwrap()).to_string();
    } else {
        warn!("Unknown architecture pointer width");
        num_string = (num.to_i32().unwrap()).to_string();
    }
    let num_bytes_vec = num_string.as_bytes().to_vec();
    num_bytes_vec
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bytes_to_int_positive() {
        let int_result: i32 = bytes_to_int(&[b'1', b'2', b'3']).unwrap();
        assert_eq!(int_result, 123);
    }

    #[test]
    fn test_bytes_to_int_negative() {
        let int_result: i32 = bytes_to_int(&[b'-', b'1', b'2', b'3']).unwrap();
        assert_eq!(int_result, -123);
    }

    #[test]
    fn test_int_to_bytes_positive() {
        let x_bytes = int_to_bytes(123i32);
        let check = vec![b'1', b'2', b'3'];
        for i in 0..x_bytes.len() {
            assert_eq!(x_bytes[i], check[i]);
        }
    }

    #[test]
    fn test_int_to_bytes_negative() {
        let x_bytes = int_to_bytes(-123i32);
        let check = vec![b'-', b'1', b'2', b'3'];
        for i in 0..x_bytes.len() {
            assert_eq!(x_bytes[i], check[i]);
        }
    }
}
