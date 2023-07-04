#![allow(dead_code)]
use num_traits::Float;

// linear mapping between two ranges, similar to arduino's map function
pub fn map(x: u32, xmin: u32, xmax: u32, ymin: u32, ymax: u32) -> u32 {
    x * (ymax - ymin) / (xmax - xmin) + ymin
}

// checks if two floats are within some threshold of each of other
pub fn almost_equal<T: Float>(d1: T, d2: T, epsilon: T) -> bool {
    (d1 - d2).abs() < epsilon
}
