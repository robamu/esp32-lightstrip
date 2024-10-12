#![allow(dead_code)]

use crate::LED_FINE_TICKER_FREQ;

#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct Scaling {
    pub current_ticks: u32,
    pub total_ticks: u32,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct FrequencyParamsMs {
    pub current: u32,
    pub min: u32,
    pub max: u32,
    pub increment_decrement: u32,
}

pub trait AdjustableSpeed {
    fn current_freq_ms(&mut self) -> u32;
    fn decrease_speed(&mut self);
    fn increase_speed(&mut self);
}

impl FrequencyParamsMs {
    pub const fn new(current: u32, min: u32, max: u32, increment_decrement: u32) -> Self {
        Self {
            current,
            min,
            max,
            increment_decrement,
        }
    }

    pub fn increment(&mut self) {
        self.current += self.increment_decrement;
        if self.current > self.max {
            self.current = self.max;
        }
    }

    pub fn decrement(&mut self) {
        self.current = self.current.saturating_sub(self.increment_decrement);
        if self.current < self.min {
            self.current = self.min;
        }
    }
}

impl Scaling {
    pub fn new(frequency_ms: u32, fine_ticks_ms: u32) -> Self {
        Self {
            current_ticks: 0,
            total_ticks: Self::total_ticks_for_pulse_frequency(frequency_ms, fine_ticks_ms),
        }
    }

    pub fn update_frequency(&mut self, frequency_ms: u32) {
        self.total_ticks =
            Self::total_ticks_for_pulse_frequency(frequency_ms, LED_FINE_TICKER_FREQ as u32);
    }

    pub fn increment(&mut self) {
        self.current_ticks += 1;
        if self.current_ticks >= self.total_ticks {
            self.current_ticks = 0;
        }
    }

    pub fn total_ticks_for_pulse_frequency(frequency_ms: u32, fine_ticks_ms: u32) -> u32 {
        frequency_ms / fine_ticks_ms
    }
}

pub mod pulse {
    use libm::floorf;

    use super::*;

    pub const DEFAULT_FREQUENCY_MS: u32 = 5000;
    pub const MIN_FREQ: u32 = 100;
    pub const MAX_FREQ: u32 = 10000;
    pub const PULSE_FREQUENCY_INCREMENT_MS: u32 = 200;

    #[derive(Debug, Copy, Clone)]
    pub struct Params {
        pub scaling: Scaling,
        pub frequency_ms: FrequencyParamsMs,
    }

    impl Params {
        pub fn new(frequency_ms: u32) -> Self {
            Self {
                scaling: Scaling::new(frequency_ms, LED_FINE_TICKER_FREQ as u32),
                frequency_ms: FrequencyParamsMs::new(
                    DEFAULT_FREQUENCY_MS,
                    MIN_FREQ,
                    MAX_FREQ,
                    PULSE_FREQUENCY_INCREMENT_MS,
                ),
            }
        }

        pub fn increment(&mut self) {
            self.scaling.increment();
        }

        pub fn brightness_for_pulse(&self) -> u8 {
            const BASE_OFFSET: f32 = 10.0;
            if self.scaling.current_ticks < self.scaling.total_ticks / 2 {
                (BASE_OFFSET
                    + floorf(
                        (self.scaling.current_ticks as f32 / (self.scaling.total_ticks / 2) as f32)
                            * (255.0 - BASE_OFFSET),
                    )) as u8
            } else {
                let ticks_in_fall = self.scaling.current_ticks - self.scaling.total_ticks / 2;
                (BASE_OFFSET
                    + floorf(
                        (1.0 - ticks_in_fall as f32 / (self.scaling.total_ticks / 2) as f32)
                            * (255.0 - BASE_OFFSET),
                    )) as u8
            }
        }
    }

    impl AdjustableSpeed for Params {
        fn current_freq_ms(&mut self) -> u32 {
            self.frequency_ms.current
        }

        fn decrease_speed(&mut self) {
            self.frequency_ms.increment();
            self.scaling.update_frequency(self.frequency_ms.current);
        }

        fn increase_speed(&mut self) {
            self.frequency_ms.decrement();
            self.scaling.update_frequency(self.frequency_ms.current);
        }
    }
}

pub mod rainbow {
    use libm::floorf;

    use super::*;

    // Frequency of full rainbow cycle.
    pub const DEFAULT_FREQUENCY_MS: u32 = 10000;
    pub const MIN_FREQ: u32 = 500;
    pub const MAX_FREQ: u32 = 25000;
    pub const FREQ_INC_DEC: u32 = 500;

    #[derive(Debug, Copy, Clone)]
    pub struct Params {
        pub scaling: Scaling,
        pub frequency_ms: FrequencyParamsMs,
    }

    impl Params {
        pub fn new(frequency_ms: u32, fine_ticks_ms: u32) -> Self {
            Self {
                scaling: Scaling::new(frequency_ms, fine_ticks_ms),
                frequency_ms: FrequencyParamsMs::new(
                    DEFAULT_FREQUENCY_MS,
                    MIN_FREQ,
                    MAX_FREQ,
                    FREQ_INC_DEC,
                ),
            }
        }

        pub fn hue(&self) -> u8 {
            floorf(self.scaling.current_ticks as f32 / self.scaling.total_ticks as f32 * 255.0)
                as u8
        }

        pub fn increment(&mut self) {
            self.scaling.increment();
        }
    }

    impl AdjustableSpeed for Params {
        fn current_freq_ms(&mut self) -> u32 {
            self.frequency_ms.current
        }

        fn decrease_speed(&mut self) {
            self.frequency_ms.increment();
            self.scaling.update_frequency(self.frequency_ms.current);
        }

        fn increase_speed(&mut self) {
            self.frequency_ms.decrement();
            self.scaling.update_frequency(self.frequency_ms.current);
        }
    }
}

pub mod moving_rainbow {
    use esp_hal::time;
    use infrared::receiver::time::InfraMonotonic;

    use super::{AdjustableSpeed, FrequencyParamsMs};

    // Frequency of full rainbow cycle for one LED.
    pub const DEFAULT_FREQUENCY_MS: u32 = 10000;
    pub const MIN_FREQ: u32 = 500;
    pub const MAX_FREQ: u32 = 30000;
    pub const FREQ_INC_DEC: u32 = 500;

    #[derive(Debug, Copy, Clone)]
    pub struct Params {
        pub current_start_hue: u8,
        pub last_hue_increment: time::Instant,
        pub ms_per_hue_inc: u32,
        pub frequency_ms: FrequencyParamsMs,
    }
    impl Default for Params {
        fn default() -> Self {
            Self {
                current_start_hue: Default::default(),
                last_hue_increment: time::Instant::ZERO_INSTANT,
                ms_per_hue_inc: (DEFAULT_FREQUENCY_MS / 255),
                frequency_ms: FrequencyParamsMs::new(
                    DEFAULT_FREQUENCY_MS,
                    MIN_FREQ,
                    MAX_FREQ,
                    FREQ_INC_DEC,
                ),
            }
        }
    }

    impl AdjustableSpeed for Params {
        fn current_freq_ms(&mut self) -> u32 {
            self.frequency_ms.current
        }

        fn decrease_speed(&mut self) {
            self.frequency_ms.increment();
            self.ms_per_hue_inc = self.frequency_ms.current / 255;
        }

        fn increase_speed(&mut self) {
            self.frequency_ms.decrement();
            self.ms_per_hue_inc = self.frequency_ms.current / 255;
        }
    }
}

pub mod disco {
    use esp_hal::time;
    use infrared::receiver::time::InfraMonotonic;

    use super::{AdjustableSpeed, FrequencyParamsMs};

    // Frequency of randomization.
    pub const DEFAULT_FREQUENCY_MS: u32 = 500;
    pub const MIN_FREQ: u32 = 40;
    pub const MAX_FREQ: u32 = 3000;
    pub const FREQ_INC_DEC: u32 = 100;

    #[derive(Debug, Copy, Clone)]
    pub struct Params {
        pub frequency_ms: FrequencyParamsMs,
        pub last_randomization: time::Instant,
    }

    impl Default for Params {
        fn default() -> Self {
            Self {
                frequency_ms: FrequencyParamsMs::new(
                    DEFAULT_FREQUENCY_MS,
                    MIN_FREQ,
                    MAX_FREQ,
                    FREQ_INC_DEC,
                ),
                last_randomization: time::Instant::ZERO_INSTANT,
            }
        }
    }

    impl AdjustableSpeed for Params {
        fn current_freq_ms(&mut self) -> u32 {
            self.frequency_ms.current
        }

        fn decrease_speed(&mut self) {
            self.frequency_ms.increment();
        }

        fn increase_speed(&mut self) {
            self.frequency_ms.decrement();
        }
    }
}
