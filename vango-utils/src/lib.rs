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
///
/// Example:
/// ```
/// use vango_utils::bytes_to_int;
/// let x1: i16 = bytes_to_int(&[b'-', b'1', b'2']).unwrap();
/// let x2: i16 = bytes_to_int(&[b'1', b'2']).unwrap();
/// assert_eq!(x1, -12);
/// assert_eq!(x2, 12);
/// ```
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
        warn!("Unknown architecture pointer width, defaulting to 32bit integer");
        num_string = (num.to_i32().unwrap()).to_string();
    }
    let num_bytes_vec = num_string.as_bytes().to_vec();
    num_bytes_vec
}

// Convert an f32 to its ASCII representation
pub fn f32_to_ascii(value: f32) -> Vec<u8> {
    let mut result = Vec::new();
    let mut value_str = value.to_string();

    if value < 0.0 {
        result.push(b'-');
        value_str = value_str[1..].to_string(); // Remove the negative sign from the string
    }
    result.extend(value_str.bytes());

    result
}

/// Converts a Vec<u8> of ASCII characters to an f32
pub fn ascii_to_f32(ascii_array: Vec<u8>) -> Result<f32, &'static str> {
    let ascii_zero = b'0';
    let ascii_decimal_point = b'.';

    let mut value_str = String::new();
    let mut has_decimal_point = false;

    for &byte in &ascii_array {
        if byte == ascii_decimal_point {
            if has_decimal_point {
                return Err("Multiple decimal points found");
            }
            has_decimal_point = true;
            value_str.push('.');
        } else if (byte >= ascii_zero) && (byte <= (ascii_zero + 9)) {
            value_str.push((byte - ascii_zero + b'0') as char);
        } else if byte == b'-' && value_str.is_empty() {
            value_str.push('-');
        } else {
            return Err("Invalid ASCII values in array");
        }
    }

    if value_str.is_empty() || (value_str == "-") || (value_str == ".") {
        return Err("Invalid input format");
    }

    match value_str.parse::<f32>() {
        Ok(value) => Ok(value),
        Err(_) => Err("Failed to parse as f32"),
    }
}

/// Returns true if two numbers have the same sign, otherwise false
pub fn opposite_signs<T: PartialOrd + Signed>(a: T, b: T) -> bool {
    if (a * b) >= T::zero() {
        return false;
    } else {
        return true;
    }
}

// // Normalizes any number to an arbitrary range
// // by assuming the range wraps around when going below min or above max
// double normalize( const double value, const double start, const double end )
// {
//   const double width       = end - start   ;   //
//   const double offsetValue = value - start ;   // value relative to 0
//
//   return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + start ;
//   // + start to reset back to start of original range
// }
pub fn normalize_to_range(value: f32, start: f32, end: f32) -> f32 {
    let width: f32 = end - start;
    let offset_value: f32 = value - start;
    offset_value - ((offset_value / width).floor() * width) + start
}

pub fn get_rotation_direction(current_angle: f32, target_angle: f32) -> bool {
    use std::f32::consts::PI;
    const RANGE_MIN: f32 = 0.0;
    const RANGE_MAX: f32 = 2.0 * PI;
    let a1 = normalize_to_range(current_angle, RANGE_MIN, RANGE_MAX);
    let a2 = normalize_to_range(target_angle, RANGE_MIN, RANGE_MAX);

    (a1 - a2).sin() >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_rotation_direction() {
        use std::f32::consts::PI;

        assert_eq!(get_rotation_direction(PI / 3.0, 0.0), true, "Case 1");
        assert_eq!(get_rotation_direction(-PI / 6.0, 0.0), false, "Case 2");
        assert_eq!(get_rotation_direction(3.0 * PI / 4.0, 0.0), true, "Case 3");
        assert_eq!(
            get_rotation_direction(3.0 * PI / 4.0, -3.0 * PI / 4.0),
            false,
            "Case 4"
        );
        assert_eq!(
            get_rotation_direction(-PI / 2.0, -PI / 4.0),
            false,
            "Case 5"
        );
    }

    #[test]
    fn test_normalize_to_range() {
        use std::f32::consts::PI;
        // [-180, 180]
        {
            const RANGE_MIN: f32 = -PI;
            const RANGE_MAX: f32 = PI;
            assert!(almost_equal(
                normalize_to_range(PI, RANGE_MIN, RANGE_MAX),
                -PI,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(0.0, RANGE_MIN, RANGE_MAX),
                0.0,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(3.0 * PI / 2.0, RANGE_MIN, RANGE_MAX),
                -PI / 2.0,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(-5.0 * PI / 2.0, RANGE_MIN, RANGE_MAX),
                -PI / 2.0,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(-PI / 4.0, RANGE_MIN, RANGE_MAX),
                -PI / 4.0,
                1e-6
            ));
        }

        // [0, 360]
        {
            const RANGE_MIN: f32 = 0.0;
            const RANGE_MAX: f32 = 2.0 * PI;
            assert!(almost_equal(
                normalize_to_range(-PI / 2.0, RANGE_MIN, RANGE_MAX),
                3.0 * PI / 2.0,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(0.0, RANGE_MIN, RANGE_MAX),
                0.0,
                1e-6
            ));

            assert!(almost_equal(
                normalize_to_range(-PI / 3.0, RANGE_MIN, RANGE_MAX),
                (2.0 * PI) - PI / 3.0,
                1e-6
            ));
        }
    }

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

    #[test]
    fn test_opposite_signs() {
        assert_eq!(opposite_signs(1.0, 1.0), false);
        assert_eq!(opposite_signs(1, 1), false);
        assert_eq!(opposite_signs(0.0, 0.0), false);
        assert_eq!(opposite_signs(0, 0), false);
        assert_eq!(opposite_signs(0, 1), false);
        assert_eq!(opposite_signs(0, -1), false);
        assert_eq!(opposite_signs(1, -1), true);
        assert_eq!(opposite_signs(1.1, -1.1), true);
    }

    #[test]
    fn test_f32_to_ascii() {
        let v1: f32 = 3.14;
        let ascii_arr1: Vec<u8> = f32_to_ascii(v1);
        assert_eq!(ascii_arr1[0], b'3');
        assert_eq!(ascii_arr1[1], b'.');
        assert_eq!(ascii_arr1[2], b'1');
        assert_eq!(ascii_arr1[3], b'4');

        let v: f32 = -3.14;
        let ascii_arr: Vec<u8> = f32_to_ascii(v);
        assert_eq!(ascii_arr[0], b'-');
        assert_eq!(ascii_arr[1], b'3');
        assert_eq!(ascii_arr[2], b'.');
        assert_eq!(ascii_arr[3], b'1');
        assert_eq!(ascii_arr[4], b'4');
    }

    #[test]
    fn test_ascii_to_f32() {
        let ascii_arr = vec![b'3', b'.', b'1', b'4'];
        let v = ascii_to_f32(ascii_arr).unwrap();
        assert!(almost_equal(v, 3.14, 1e-6));

        let ascii_arr = vec![b'-', b'3', b'.', b'1', b'4'];
        let v = ascii_to_f32(ascii_arr).unwrap();
        assert!(almost_equal(v, -3.14, 1e-6));
    }
}
