// Code taken from https://github.com/esp-rs/esp-idf-hal/blob/master/examples/pcnt_i64_encoder.rs

#![allow(dead_code)]
use std::cmp::min;
use std::sync::atomic::AtomicI64;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::pcnt::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::EspError;

use core::f32::consts::PI as pi;

const LOW_LIMIT: i16 = -100;
const HIGH_LIMIT: i16 = 100;

pub const ENCODER_RATE_MS: u64 = 10; // 100 Hz
pub const GEAR_RATIO: u64 = 50;
pub const COUNTS_PER_REV: u64 = 7; // counts per revolution
pub const TICKS_TO_RPS: f32 = 1000.0 / (COUNTS_PER_REV * GEAR_RATIO * ENCODER_RATE_MS * 4) as f32;
pub const TICKS_TO_RPM: f32 =
    60.0 * 1000.0 / (COUNTS_PER_REV * GEAR_RATIO * ENCODER_RATE_MS * 4) as f32;
pub const TICKS_TO_RAD_PER_SEC: f32 =
    pi * 1000.0 / (COUNTS_PER_REV * GEAR_RATIO * ENCODER_RATE_MS * 4) as f32;
pub const TICKS_PER_RAD: f32 = (7.0 * 50.0 * 4.0) / (2.0 * pi);

pub struct Encoder<'d> {
    unit: PcntDriver<'d>,
    approx_value: Arc<AtomicI64>,
}

impl<'d> Encoder<'d> {
    pub fn new<PCNT: Pcnt>(
        pcnt: impl Peripheral<P = PCNT> + 'd,
        pin_a: impl Peripheral<P = impl InputPin> + 'd,
        pin_b: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self, EspError> {
        let mut unit = PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;
        unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;
        unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;

        unit.set_filter_value(min(10 * 80, 1023))?;
        unit.filter_enable()?;

        let approx_value = Arc::new(AtomicI64::new(0));
        // unsafe interrupt code to catch the upper and lower limits from the encoder
        // and track the overflow in `value: Arc<AtomicI64>` - I plan to use this for
        // a wheeled robot's odomerty
        unsafe {
            let approx_value = approx_value.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i64, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i64, Ordering::SeqCst);
                }
            })?;
        }
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?;
        unit.counter_pause()?;
        unit.counter_clear()?;
        unit.counter_resume()?;

        Ok(Self { unit, approx_value })
    }

    pub fn get_value(&self) -> Result<i64, EspError> {
        let value =
            self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i64;
        Ok(value)
    }
}
