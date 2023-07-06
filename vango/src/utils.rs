#![allow(dead_code)]
use num_traits::Float;
use num_traits::PrimInt;

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
