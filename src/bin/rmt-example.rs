#![no_std]
#![no_main]

use core::slice::IterMut;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::Clocks,
    gpio::{self, Io, Level, Output, OutputConfig},
    rmt::{PulseCode, Rmt, TxChannelAsync, TxChannelConfig, TxChannelCreatorAsync},
    time::Rate,
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

    let _io = Io::new(peripherals.IO_MUX);

    let switch_cfg = OutputConfig::default().with_drive_strength(gpio::DriveStrength::_5mA);
    let mut lightstrip_switch = Output::new(peripherals.GPIO1, Level::Low, switch_cfg);
    lightstrip_switch.set_low();

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap()
        .into_async();

    let channel_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_idle_output(true);
    let mut channel = rmt
        .channel0
        .configure(peripherals.GPIO0, channel_config)
        .unwrap();

    // Assume the RMT peripheral is set up to use the APB clock
    let clocks = Clocks::get();
    let src_clock = clocks.apb_clock.as_mhz();

    let pulse0 = u32::new(
        Level::High,
        ((SK68XX_T0H_NS * src_clock) / 1000) as u16,
        Level::Low,
        ((SK68XX_T0L_NS * src_clock) / 1000) as u16,
    );
    let pulse1 = u32::new(
        Level::High,
        ((SK68XX_T1H_NS * src_clock) / 1000) as u16,
        Level::Low,
        ((SK68XX_T1L_NS * src_clock) / 1000) as u16,
    );

    let pulses = (pulse0, pulse1);
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
