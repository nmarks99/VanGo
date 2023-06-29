// Code adapted from https://github.com/esp-rs/esp-idf-hal/blob/master/examples/rmt_neopixel.rs

use anyhow;
use core::ops::Mul;
use core::time::Duration;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::rmt;
use esp_idf_hal::rmt::PinState;
use esp_idf_hal::rmt::RmtChannel;
use esp_idf_hal::rmt::TxRmtDriver;
use esp_idf_hal::rmt::{FixedLengthSignal, Pulse};
use esp_idf_sys::EspError;

pub struct Neopixel<'a> {
    tx_driver: TxRmtDriver<'a>,
}

impl<'a> Neopixel<'a> {
    pub fn new(
        pin: impl Peripheral<P = impl OutputPin> + 'a,
        rmt_channel: impl Peripheral<P = impl RmtChannel> + 'a
    ) -> Result<Self, EspError> {
        let rmt_config = rmt::config::TransmitConfig::new().clock_divider(1);
        let rmt_tx = TxRmtDriver::new(rmt_channel, pin, &rmt_config)?;
        Ok(Self { tx_driver: rmt_tx })
    }

    // if the provided color is not implemented, do nothing
    pub fn set_color(&mut self, color: &str, brightness: f32) -> anyhow::Result<()> {
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
            self.set_rgb(rgb_op.unwrap(), brightness)?;
        }
        else {
            println!("Warning: Color {} not implemented for Neopixel", color);
        }
        Ok(())
    }

    // sets the neopixel to the specified RGB
    pub fn set_rgb(&mut self, rgb: Rgb, brightness: f32) -> anyhow::Result<()> {
        let rgb = rgb * brightness;
        let color: u32 = rgb.into();
        let ticks_hz = self.tx_driver.counter_clock()?;
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
        self.tx_driver.start_blocking(&signal)?;
        Ok(())
    }


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

impl Mul<f32> for Rgb {
    // allows for multiplication of a Rgb by an f32,
    // which simply multiplies each of the r, g, and b
    // values by the f32 and converts back to u8
    type Output = Rgb;

    fn mul(self, rhs: f32) -> Rgb {
        Rgb {
            r: (self.r as f32 * rhs) as u8,
            g: (self.g as f32 * rhs) as u8,
            b: (self.b as f32 * rhs) as u8,
        }
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
