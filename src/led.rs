#![allow(dead_code)]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Ticker;
use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, Output, OutputConfig},
    rmt::{ChannelCreator, Rmt},
    rng::Rng,
    time::Instant,
    Async,
};
use libm::floorf;
use log::info;
use smart_leds::{colors, hsv, SmartLedsWriteAsync, RGB8};

use crate::conf::{self, LED_CMD_CHANNEL_DEPTH};

pub static LED_CHANNEL: Channel<CriticalSectionRawMutex, LedCmd, LED_CMD_CHANNEL_DEPTH> =
    Channel::new();

const COLOR_SET: [RGB8; 14] = [
    colors::WHITE,
    colors::YELLOW,
    colors::ORANGE,
    colors::ORANGE_RED,
    colors::RED,
    colors::DARK_RED,
    colors::PURPLE,
    colors::HOT_PINK,
    colors::DARK_BLUE,
    colors::BLUE,
    colors::SKY_BLUE,
    colors::DARK_GREEN,
    colors::GREEN,
    colors::YELLOW_GREEN,
];

#[derive(Debug, Default, PartialEq, Eq, defmt::Format)]
pub enum LightMode {
    #[default]
    Rainbow,
    OneColor,
    Pulsing,
    MovingRainbow,
    Disco,
}

#[derive(Debug, Default, PartialEq, Eq)]
pub enum Mode {
    #[default]
    Off,
    On(LightMode),
}

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

#[derive(Debug, PartialEq, Eq)]
pub enum LedCmd {
    Switch,
    SelectMode(LightMode),
    IncreaseBrightness,
    DecreaseBrightness,
    IncreaseSpeed,
    DecreaseSpeed,
    NextColor,
    PrevColor,
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
            Self::total_ticks_for_pulse_frequency(frequency_ms, conf::LED_FINE_TICKER_FREQ as u32);
        self.current_ticks = 0;
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

pub struct Ledstrip {
    mode: Mode,
    led_was_switched_in_cycle: bool,
    color_was_changed_in_cycle: bool,
    brightness: u8,
    color_index: usize,
    rainbow_parameters: rainbow::Params,
    pulse_parameters: pulse::Params,
    moving_rainbow_parameters: moving_rainbow::Params,
    disco_params: disco::Params,
}

impl Default for Ledstrip {
    fn default() -> Self {
        Self {
            mode: Mode::Off,
            color_index: 0,
            led_was_switched_in_cycle: false,
            color_was_changed_in_cycle: false,
            brightness: conf::DEFAULT_BRIGHTNESS,
            rainbow_parameters: rainbow::Params::new(
                rainbow::DEFAULT_FREQUENCY_MS,
                conf::LED_FINE_TICKER_FREQ as u32,
            ),
            pulse_parameters: pulse::Params::new(pulse::DEFAULT_FREQUENCY_MS),
            moving_rainbow_parameters: Default::default(),
            disco_params: Default::default(),
        }
    }
}

#[derive(Debug)]
pub enum CmdResult {
    SwitchToOff,
    SwitchToOn,
    Other,
}

impl Ledstrip {
    fn all_commands_were_handled(&mut self) {
        self.led_was_switched_in_cycle = false;
        self.color_was_changed_in_cycle = false;
    }

    fn handle_led_cmd(
        &mut self,
        led_cmd: LedCmd,
        switch_pin: &mut Output<'static>,
        fine_ticker: &mut Ticker,
    ) -> CmdResult {
        let mut cmd_result = CmdResult::Other;
        match led_cmd {
            LedCmd::Switch => match self.mode {
                Mode::Off => {
                    // We only allow one switch per cycle.
                    if !self.led_was_switched_in_cycle {
                        switch_pin.set_high();
                        self.mode = Mode::On(LightMode::default());
                        self.led_was_switched_in_cycle = true;
                        fine_ticker.reset();
                        info!("switching lightstrip on");
                        cmd_result = CmdResult::SwitchToOn;
                    }
                }
                Mode::On { .. } => {
                    if !self.led_was_switched_in_cycle {
                        switch_pin.set_low();
                        self.mode = Mode::Off;
                        self.led_was_switched_in_cycle = true;
                        info!("switching lightstrip off");
                        cmd_result = CmdResult::SwitchToOff;
                    }
                }
            },
            LedCmd::SelectMode(light_mode_cmd) => {
                match light_mode_cmd {
                    LightMode::Rainbow => {
                        info!("rainbow mode");
                        self.mode = Mode::On(LightMode::Rainbow);
                    }
                    LightMode::OneColor => {
                        info!("one color");
                        self.mode = Mode::On(LightMode::OneColor);
                    }
                    LightMode::Pulsing => {
                        info!("pulse mode");
                        self.mode = Mode::On(LightMode::Pulsing);
                    }
                    LightMode::MovingRainbow => {
                        info!("moving rainbow");
                        self.mode = Mode::On(LightMode::MovingRainbow);
                    }
                    LightMode::Disco => {
                        info!("disco");
                        self.mode = Mode::On(LightMode::Disco);
                    }
                }
                // Always reset the fine ticker when changing modes to avoid weird bugs
                // when switching from mode which require the fine ticker to modes which do not.
                fine_ticker.reset();
            }
            LedCmd::IncreaseBrightness => {
                info!("increasing brightness");
                self.brightness = self.brightness.saturating_add(10);
                info!(
                    "current brightness: {} %",
                    (self.brightness as f32 / 255_f32) * 100.0
                );
            }
            LedCmd::DecreaseBrightness => {
                info!("decreasing brightness");
                self.brightness = self.brightness.saturating_sub(10);
                info!(
                    "current brightness: {} %",
                    (self.brightness as f32 / 255_f32) * 100.0
                );
            }
            LedCmd::IncreaseSpeed => {
                info!("increasing speed");
                let current_freq = match self.mode {
                    Mode::On(LightMode::Rainbow) => {
                        self.rainbow_parameters.increase_speed();
                        self.rainbow_parameters.current_freq_ms()
                    }
                    Mode::On(LightMode::MovingRainbow) => {
                        self.moving_rainbow_parameters.increase_speed();
                        self.moving_rainbow_parameters.current_freq_ms()
                    }
                    Mode::On(LightMode::Disco) => {
                        self.disco_params.increase_speed();
                        self.disco_params.current_freq_ms()
                    }
                    Mode::On(LightMode::Pulsing) => {
                        self.pulse_parameters.increase_speed();
                        self.pulse_parameters.current_freq_ms()
                    }
                    _ => {
                        info!("speed increase not supported for current mode");
                        0
                    }
                };
                if current_freq > 0 {
                    info!("current frequency: {} ms", current_freq);
                }
            }
            LedCmd::DecreaseSpeed => {
                info!("decreasing speed");

                let current_freq = match self.mode {
                    Mode::On(LightMode::Rainbow) => {
                        self.rainbow_parameters.decrease_speed();
                        self.rainbow_parameters.current_freq_ms()
                    }
                    Mode::On(LightMode::MovingRainbow) => {
                        self.moving_rainbow_parameters.decrease_speed();
                        self.moving_rainbow_parameters.current_freq_ms()
                    }
                    Mode::On(LightMode::Disco) => {
                        self.disco_params.decrease_speed();
                        self.disco_params.current_freq_ms()
                    }
                    Mode::On(LightMode::Pulsing) => {
                        self.pulse_parameters.decrease_speed();
                        self.pulse_parameters.current_freq_ms()
                    }
                    _ => {
                        info!("speed decrease not supported for current mode");
                        0
                    }
                };

                if current_freq > 0 {
                    info!("current frequency: {} ms", current_freq);
                }
            }
            LedCmd::PrevColor => {
                if !self.color_was_changed_in_cycle {
                    if self.color_index == 0 {
                        self.color_index = COLOR_SET.len() - 1;
                    } else {
                        self.color_index -= 1;
                    }
                    self.color_was_changed_in_cycle = true;
                }
            }
            LedCmd::NextColor => {
                if !self.color_was_changed_in_cycle {
                    if self.color_index == COLOR_SET.len() - 1 {
                        self.color_index = 0;
                    } else {
                        self.color_index += 1;
                    }
                    self.color_was_changed_in_cycle = true;
                }
            }
        }
        cmd_result
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
                scaling: Scaling::new(frequency_ms, conf::LED_FINE_TICKER_FREQ as u32),
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
    use libm::roundf;

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
            roundf((self.scaling.current_ticks as f32 / self.scaling.total_ticks as f32) * 255.0)
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
    use esp_hal::time::{self, Instant};

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
                last_hue_increment: Instant::EPOCH,
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
    use esp_hal::time::{self, Instant};

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
                last_randomization: Instant::EPOCH,
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

#[embassy_executor::task]
pub async fn led_task(
    rmt: Rmt<'static, Async>,
    mut rng: Rng,
    led_pin: Output<'static>,
    mut switch_pin: Output<'static>,
) {
    let rmt_buffer = [0u32; esp_hal_smartled::buffer_size_async(conf::NUM_LEDS)];
    let mut ledstrip = Ledstrip::default();
    let receiver = LED_CHANNEL.receiver();
    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led_adapter =
        esp_hal_smartled::SmartLedsAdapterAsync::new(rmt.channel0, led_pin, rmt_buffer);
    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(
        conf::LED_CMD_CHECK_FREQ_MS,
    ));
    let mut fine_ticker = Ticker::every(embassy_time::Duration::from_millis(
        conf::LED_FINE_TICKER_FREQ,
    ));
    let divisor = conf::LED_CMD_CHECK_FREQ_MS / conf::LED_FINE_TICKER_FREQ;
    let mut data: [RGB8; conf::NUM_LEDS] = [colors::WHITE; conf::NUM_LEDS];
    let mut color = hsv::Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    loop {
        while let Ok(led_cmd) = receiver.try_receive() {
            match ledstrip.handle_led_cmd(led_cmd, &mut switch_pin, &mut fine_ticker) {
                CmdResult::SwitchToOff => {
                    let led_pin = unsafe { AnyPin::steal(0) };
                    Input::new(led_pin, InputConfig::default());
                }
                CmdResult::SwitchToOn => {
                    let led_pin = unsafe { AnyPin::steal(0) };
                    let output_pin =
                        Output::new(led_pin, esp_hal::gpio::Level::Low, OutputConfig::default());
                    let channel_0 = unsafe { ChannelCreator::<Async, 0>::steal() };
                    drop(led_adapter);
                    led_adapter = esp_hal_smartled::SmartLedsAdapterAsync::new(
                        channel_0, output_pin, rmt_buffer,
                    );
                }
                CmdResult::Other => (),
            }
        }
        ledstrip.all_commands_were_handled();
        match &mut ledstrip.mode {
            Mode::Off => {
                ticker.next().await;
            }
            Mode::On(mode) => {
                match mode {
                    LightMode::Pulsing => {
                        data = [COLOR_SET[ledstrip.color_index]; conf::NUM_LEDS];
                        for _ in 0..=divisor {
                            led_adapter
                                .write(smart_leds::brightness(
                                    smart_leds::gamma(data.iter().cloned()),
                                    ledstrip.pulse_parameters.brightness_for_pulse(),
                                ))
                                .await
                                .unwrap();
                            ledstrip.pulse_parameters.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightMode::OneColor => {
                        data = [COLOR_SET[ledstrip.color_index]; conf::NUM_LEDS];
                        led_adapter
                            .write(smart_leds::brightness(
                                smart_leds::gamma(data.iter().cloned()),
                                ledstrip.brightness,
                            ))
                            .await
                            .unwrap();
                        ticker.next().await;
                    }
                    LightMode::Rainbow => {
                        // Iterate over the rainbow!
                        for _ in 0..=divisor {
                            color.hue = ledstrip.rainbow_parameters.hue();
                            // Convert from the HSV color space (where we can easily transition from one
                            // color to the other) to the RGB color space that we can then send to the LED
                            data = [hsv::hsv2rgb(color); conf::NUM_LEDS];
                            // When sending to the LED, we do a gamma correction first (see smart_leds
                            // documentation for details).
                            led_adapter
                                .write(smart_leds::brightness(
                                    smart_leds::gamma(data.iter().cloned()),
                                    ledstrip.brightness,
                                ))
                                .await
                                .unwrap();
                            ledstrip.rainbow_parameters.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightMode::Disco => {
                        for _ in 0..=divisor {
                            let now = Instant::now();
                            if (now - ledstrip.disco_params.last_randomization).as_millis()
                                > ledstrip.disco_params.frequency_ms.current as u64
                            {
                                for next_rgb in data.iter_mut() {
                                    color.hue = libm::roundf(
                                        (rng.random() as f32 / u32::MAX as f32) * 255.0,
                                    ) as u8;
                                    *next_rgb = hsv::hsv2rgb(color);
                                }
                                led_adapter
                                    .write(smart_leds::brightness(
                                        smart_leds::gamma(data.iter().cloned()),
                                        ledstrip.brightness,
                                    ))
                                    .await
                                    .unwrap();
                                ledstrip.disco_params.last_randomization = now;
                            }
                            fine_ticker.next().await;
                        }
                    }
                    LightMode::MovingRainbow => {
                        for _ in 0..=divisor {
                            for (idx, next_rgb) in data.iter_mut().enumerate() {
                                if idx == 0 {
                                    color.hue =
                                        ledstrip.moving_rainbow_parameters.current_start_hue;
                                } else {
                                    color.hue = (ledstrip
                                        .moving_rainbow_parameters
                                        .current_start_hue
                                        as u16
                                        + floorf(idx as f32 / conf::NUM_LEDS as f32 * 255.0) as u16
                                            % 255)
                                        as u8;
                                }
                                *next_rgb = hsv::hsv2rgb(color);
                            }
                            // Convert from the HSV color space (where we can easily transition from one
                            // color to the other) to the RGB color space that we can then send to the LED
                            led_adapter
                                .write(smart_leds::brightness(
                                    smart_leds::gamma(data.iter().cloned()),
                                    ledstrip.brightness,
                                ))
                                .await
                                .unwrap();

                            if ledstrip.moving_rainbow_parameters.ms_per_hue_inc
                                > conf::LED_FINE_TICKER_FREQ as u32
                            {
                                let now = esp_hal::time::Instant::now();
                                if (now - ledstrip.moving_rainbow_parameters.last_hue_increment)
                                    .as_millis()
                                    > ledstrip.moving_rainbow_parameters.ms_per_hue_inc as u64
                                {
                                    ledstrip.moving_rainbow_parameters.current_start_hue = ledstrip
                                        .moving_rainbow_parameters
                                        .current_start_hue
                                        .wrapping_add(1);
                                    ledstrip.moving_rainbow_parameters.last_hue_increment = now;
                                }
                            } else {
                                let mut increment = conf::LED_FINE_TICKER_FREQ as u32
                                    / ledstrip.moving_rainbow_parameters.ms_per_hue_inc;
                                if increment > 255 {
                                    increment = 255
                                }
                                // Might need to increment more than 1 hue
                                ledstrip.moving_rainbow_parameters.current_start_hue = ledstrip
                                    .moving_rainbow_parameters
                                    .current_start_hue
                                    .wrapping_add(increment as u8);
                            }
                            fine_ticker.next().await;
                        }
                    }
                }
            }
        }
    }
}
