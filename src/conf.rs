#[cfg(feature = "bedroom")]
pub const NUM_LEDS: usize = 46;
#[cfg(feature = "tree")]
pub const NUM_LEDS: usize = 50;

pub const DEFAULT_BRIGHTNESS: u8 = 80;

pub const LED_CMD_CHECK_FREQ_MS: u64 = 200;
pub const LED_FINE_TICKER_FREQ: u64 = 20;

pub const LED_CMD_CHANNEL_DEPTH: usize = 24;
