#![allow(dead_code)]
use num_traits::Float;
use num_traits::{PrimInt, Signed};

// linear mapping between two ranges, similar to arduino's map function
pub fn map<T: PrimInt>(x: T, xmin: T, xmax: T, ymin: T, ymax: T) -> T {
    assert!(xmin < xmax, "xmin must be less than xmax");
    let x_ratio = (x - xmin).to_f32().unwrap() / (xmax - xmin).to_f32().unwrap();
    let res = ymin.to_f32().unwrap() + x_ratio * (ymax - ymin).to_f32().unwrap();
    T::from(res).expect("conversion failed")
}

// checks if two floats are within some threshold of each of other
pub fn almost_equal<T: Float>(d1: T, d2: T, epsilon: T) -> bool {
    (d1 - d2).abs() < epsilon
}

// pub fn bytes_to_int<T: PrimInt>(arr: &[u8]) -> Option<T> {
//     let mut result = T::zero();
//     for &byte in arr {
//         if byte >= b'0' && byte <= b'9' {
//             result =
//                 result * T::from(10).unwrap() + (T::from(byte).unwrap() - T::from(b'0').unwrap());
//         } else {
//             return None;
//         }
//     }
//     Some(result)
// }

/// Converts an array of bytes to a signed integer
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
