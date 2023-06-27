// Code taken from https://github.com/esp-rs/esp-idf-hal/blob/master/examples/rmt_neopixel.rs

use anyhow;
use core::time::Duration;
use esp_idf_hal::rmt::PinState;
use esp_idf_hal::rmt::TxRmtDriver;
use esp_idf_hal::rmt::{FixedLengthSignal, Pulse};

// sets the neopixel to the specified RGB
pub fn set_rgb(rgb: Rgb, tx: &mut TxRmtDriver) -> anyhow::Result<()> {
    let color: u32 = rgb.into();
    let ticks_hz = tx.counter_clock()?;
    let (t0h, t0l, t1h, t1l) = (
        Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(350))?,
        Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(800))?,
        Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(700))?,
        Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(600))?,
    );
    let mut signal = FixedLengthSignal::<24>::new();
    for i in (0..24).rev() {
        let p = 2_u32.pow(i);
        let bit: bool = p & color != 0;
        let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
        signal.set(23 - i as usize, &(high_pulse, low_pulse))?;
    }
    tx.start_blocking(&signal)?;
    Ok(())
}

// set color from a string
// if the provided color is not implemented, do nothing
pub fn set_color(color: &str, tx: &mut TxRmtDriver) -> anyhow::Result<()> {
    let rgb_op: Option<Rgb> = match color {
        "red" => Some(Rgb::new(255, 0, 0)),
        "green" => Some(Rgb::new(0, 255, 0)),
        "blue" => Some(Rgb::new(0, 0, 255)),
        "purple" => Some(Rgb::new(255, 0, 255)),
        "yellow" => Some(Rgb::new(255, 255, 0)),
        "cyan" => Some(Rgb::new(0, 255, 255)),
        "white" => Some(Rgb::new(50, 50, 50)),
        _ => None,
    };
    if rgb_op.is_some() {
        set_rgb(rgb_op.unwrap(), tx)?;
    }
    Ok(())
}

pub struct Rgb {
    r: u8,
    g: u8,
    b: u8,
}

impl Rgb {
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
    /// Converts hue, saturation, value to RGB
    pub fn from_hsv(h: u32, s: u32, v: u32) -> anyhow::Result<Self> {
        if h > 360 || s > 100 || v > 100 {
            anyhow::bail!("The given HSV values are not in valid range");
        }
        let s = s as f64 / 100.0;
        let v = v as f64 / 100.0;
        let c = s * v;
        let x = c * (1.0 - (((h as f64 / 60.0) % 2.0) - 1.0).abs());
        let m = v - c;
        let (r, g, b) = match h {
            0..=59 => (c, x, 0.0),
            60..=119 => (x, c, 0.0),
            120..=179 => (0.0, c, x),
            180..=239 => (0.0, x, c),
            240..=299 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };
        Ok(Self {
            r: ((r + m) * 255.0) as u8,
            g: ((g + m) * 255.0) as u8,
            b: ((b + m) * 255.0) as u8,
        })
    }
}

impl From<Rgb> for u32 {
    /// Convert RGB to u32 color value
    ///
    /// e.g. rgb: (1,2,4)
    /// G        R        B
    /// 7      0 7      0 7      0
    /// 00000010 00000001 00000100
    fn from(rgb: Rgb) -> Self {
        ((rgb.r as u32) << 16) | ((rgb.g as u32) << 8) | rgb.b as u32
    }
}
