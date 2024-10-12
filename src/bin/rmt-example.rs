//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use core::slice::IterMut;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::Clocks,
    gpio::{Io, Level, Output},
    prelude::*,
    rmt::{asynch::TxChannelAsync, PulseCode, Rmt, TxChannelConfig, TxChannelCreatorAsync},
    timer::timg::TimerGroup,
};
use esp_hal_smartled::LedAdapterError;
use esp_println::println;
use smart_leds::{colors, RGB8};

const SK68XX_CODE_PERIOD: u32 = 1200;
const SK68XX_T0H_NS: u32 = 320;
const SK68XX_T0L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T0H_NS;
const SK68XX_T1H_NS: u32 = 640;
const SK68XX_T1L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T1H_NS;

pub fn convert_rgb_to_pulse(
    value: RGB8,
    mut_iter: &mut IterMut<u32>,
    pulses: (u32, u32),
) -> Result<(), LedAdapterError> {
    convert_rgb_channel_to_pulses(value.g, mut_iter, pulses)?;
    convert_rgb_channel_to_pulses(value.r, mut_iter, pulses)?;
    convert_rgb_channel_to_pulses(value.b, mut_iter, pulses)?;
    *mut_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? = 0;
    Ok(())
}

fn convert_rgb_channel_to_pulses(
    channel_value: u8,
    mut_iter: &mut IterMut<u32>,
    pulses: (u32, u32),
) -> Result<(), LedAdapterError> {
    for position in [128, 64, 32, 16, 8, 4, 2, 1] {
        *mut_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? =
            match channel_value & position {
                0 => pulses.0,
                _ => pulses.1,
            }
    }

    Ok(())
}

const NUM_LEDS: usize = 46;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut lightstrip_switch = Output::new(io.pins.gpio1, Level::High);
    lightstrip_switch.set_drive_strength(esp_hal::gpio::DriveStrength::I40mA);
    lightstrip_switch.set_high();

    let rmt = Rmt::new_async(peripherals.RMT, 80.MHz()).unwrap();

    let mut channel = rmt
        .channel0
        .configure(
            io.pins.gpio0,
            TxChannelConfig {
                clk_divider: 1,
                idle_output_level: false,
                carrier_modulation: false,
                idle_output: true,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    // Assume the RMT peripheral is set up to use the APB clock
    let clocks = Clocks::get();
    let src_clock = clocks.apb_clock.to_MHz();

    let pulse0 = PulseCode {
        level1: true,
        length1: ((SK68XX_T0H_NS * src_clock) / 1000) as u16,
        level2: false,
        length2: ((SK68XX_T0L_NS * src_clock) / 1000) as u16,
    };
    let pulse1 = PulseCode {
        level1: true,
        length1: ((SK68XX_T1H_NS * src_clock) / 1000) as u16,
        level2: false,
        length2: ((SK68XX_T1L_NS * src_clock) / 1000) as u16,
    };

    let pulses = (u32::from(pulse0), u32::from(pulse1));
    let data: [RGB8; NUM_LEDS] = [colors::DARK_RED; NUM_LEDS];
    let mut rmt_buffer = [0u32; NUM_LEDS * (24 + 1)];
    let mut rmt_iter = rmt_buffer.iter_mut();
    for next_color in &data {
        convert_rgb_to_pulse(*next_color, &mut rmt_iter, pulses).unwrap();
    }
    //convert_rgb_to_pulse(data[0], &mut rmt_buffer.iter_mut(), pulses).unwrap();
    //rmt_buffer[24] = 0;
    loop {
        println!("transmit");
        for next_chunk in rmt_buffer.chunks(25) {
            channel.transmit(next_chunk).await.unwrap();
        }
        println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
