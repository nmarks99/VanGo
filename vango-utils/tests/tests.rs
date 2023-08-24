use vango_utils::*;

#[test]
fn test_linspace() {
    let x = linspace(0.0, 5.0, 10);
    println!("linspace (0,5,10): {:?}", x);
}

#[test]
fn test_arange() {
    let x = arange(0.0, 11.0, 1.0);
    println!("arange (0,11,1): {:?}", x);
}

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

#[test]
fn test_sign() {
    assert_eq!(sign(123), 1);
    assert_eq!(sign(-123), -1);

    assert_eq!(sign(123.0), 1.0);
    assert_eq!(sign(-123.0), -1.0);
}

#[test]
fn test_min_angle() {
    let pi = std::f64::consts::PI;
    let a: f64 = pi / 4.0;
    let b: f64 = -pi / 2.0;
    assert!(almost_equal(get_min_angle(a, b), 3.0 * pi / 4.0, 1e-6));
    assert!(almost_equal(get_min_angle(b, a), -3.0 * pi / 4.0, 1e-6));
}
