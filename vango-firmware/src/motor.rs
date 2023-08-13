#[derive(ParialEq)]
pub enum MotorDirection {
    Forward,
    Backward,
}

pub fn set_direction(
    pin_forward: impl Peripheral<P = impl OutputPin> + 'a,
    pin_backward: impl Peripheral<P = impl OutputPin> + 'a,
    direction: MotorDirection,
) {
    let mut left_direction = PinDriver::output(pin)?;
    let mut right_direction = PinDriver::output(pin)?;
    if direction == MotorDirection::Forward {
        left_direction.set_low()?;
        right_direction.set_high()?;
    } else if direction == MotorDirection::Backward {
        left_direction.set_high()?;
        right_direction.set_low()?;
    }
}
