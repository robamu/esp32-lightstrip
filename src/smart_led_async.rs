use core::slice::IterMut;

use esp_hal::{
    clock::Clocks,
    gpio::PeripheralOutput,
    peripheral::Peripheral,
    rmt::{asynch::TxChannelAsync, PulseCode, TxChannelConfig, TxChannelCreatorAsync},
};
use esp_hal_smartled::LedAdapterError;
use smart_leds::RGB8;

const SK68XX_CODE_PERIOD: u32 = 1200;
const SK68XX_T0H_NS: u32 = 320;
const SK68XX_T0L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T0H_NS;
const SK68XX_T1H_NS: u32 = 640;
const SK68XX_T1L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T1H_NS;

/// Adapter taking an RMT channel and a specific pin and providing RGB LED
/// interaction functionality using the `smart-leds` crate
pub struct SmartLedAdapterAsync<Tx, const BUFFER_SIZE: usize> {
    channel: Tx,
    rmt_buffer: [u32; BUFFER_SIZE],
    pulses: (u32, u32),
}

impl<'d, Tx: TxChannelAsync, const BUFFER_SIZE: usize> SmartLedAdapterAsync<Tx, BUFFER_SIZE> {
    /// Create a new adapter object that drives the pin using the RMT channel.
    pub fn new<C, O>(
        channel: C,
        pin: impl Peripheral<P = O> + 'd,
        rmt_buffer: [u32; BUFFER_SIZE],
    ) -> SmartLedAdapterAsync<Tx, BUFFER_SIZE>
    where
        O: PeripheralOutput + 'd,
        C: TxChannelCreatorAsync<'d, Tx, O>,
    {
        let config = TxChannelConfig {
            clk_divider: 1,
            idle_output_level: false,
            carrier_modulation: false,
            idle_output: true,

            ..TxChannelConfig::default()
        };

        let channel = channel.configure(pin, config).unwrap();
        // Assume the RMT peripheral is set up to use the APB clock
        let clocks = Clocks::get();
        let src_clock = clocks.apb_clock.to_MHz();

        Self {
            channel,
            rmt_buffer,
            pulses: (
                u32::from(PulseCode {
                    level1: true,
                    length1: ((SK68XX_T0H_NS * src_clock) / 1000) as u16,
                    level2: false,
                    length2: ((SK68XX_T0L_NS * src_clock) / 1000) as u16,
                }),
                u32::from(PulseCode {
                    level1: true,
                    length1: ((SK68XX_T1H_NS * src_clock) / 1000) as u16,
                    level2: false,
                    length2: ((SK68XX_T1L_NS * src_clock) / 1000) as u16,
                }),
            ),
        }
    }

    pub async fn write(
        &mut self,
        led: impl IntoIterator<Item = RGB8>,
    ) -> Result<(), LedAdapterError> {
        self.prepare_rmt_buffer(led)?;
        for chunk in self.rmt_buffer.chunks(25) {
            self.channel
                .transmit(chunk)
                .await
                .map_err(LedAdapterError::TransmissionError)?;
        }
        Ok(())
    }

    pub fn prepare_rmt_buffer(
        &mut self,
        iterator: impl IntoIterator<Item = RGB8>,
    ) -> Result<(), LedAdapterError> {
        // We always start from the beginning of the buffer
        let mut seq_iter = self.rmt_buffer.iter_mut();

        // Add all converted iterator items to the buffer.
        // This will result in an `BufferSizeExceeded` error in case
        // the iterator provides more elements than the buffer can take.
        for item in iterator {
            Self::convert_rgb_to_pulse(item, &mut seq_iter, self.pulses)?;
        }
        Ok(())
    }

    /// Converts a RGB value to the correspodnign pulse value.
    pub fn convert_rgb_to_pulse(
        value: RGB8,
        mut_iter: &mut IterMut<u32>,
        pulses: (u32, u32),
    ) -> Result<(), LedAdapterError> {
        Self::convert_rgb_channel_to_pulses(value.g, mut_iter, pulses)?;
        Self::convert_rgb_channel_to_pulses(value.r, mut_iter, pulses)?;
        Self::convert_rgb_channel_to_pulses(value.b, mut_iter, pulses)?;
        *mut_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? = 0;

        Ok(())
    }

    pub fn convert_rgb_channel_to_pulses(
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
}
