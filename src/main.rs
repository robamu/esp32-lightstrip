#![no_std]
#![no_main]

mod async_test;

use core::cell::Cell;
use core::cell::RefCell;
use core::slice::IterMut;

use critical_section::Mutex;
use defmt::println;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::InputPin;
use esp_backtrace as _;
use esp_hal::clock::Clocks;
use esp_hal::gpio;
use esp_hal::gpio::PeripheralOutput;
use esp_hal::peripheral::Peripheral;
use esp_hal::rmt::asynch::TxChannelAsync;
use esp_hal::rmt::PulseCode;
use esp_hal::rmt::Rmt;
use esp_hal::rmt::TxChannelConfig;
use esp_hal::rmt::TxChannelCreatorAsync;
use esp_hal::time;
use esp_hal::time::Instant;
use esp_hal::Async;
use esp_hal::{
    gpio::{Input, Io, Level, Output, Pull},
    prelude::*,
    timer::timg,
};
use esp_hal_smartled::LedAdapterError;
use esp_println as _;
use infrared::protocol::nec::NecCommand;
use infrared::receiver::time::InfraMonotonic;
use infrared::remotecontrol::{self, Action, RemoteControlModel};
use infrared::{protocol::Nec, receiver, Receiver};
use libm::floorf;
use libm::roundf;
use log::{debug, info, warn};
use smart_leds::colors;
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    RGB8,
};

const LED_CMD_CHECK_FREQ_MS: u64 = 200;
const LED_FINE_TICKER_FREQ: u64 = 20;

const BED_LIGHTSTRIPS_LEDS: usize = 46;
const IR_CMD_CHANNEL_DEPTH: usize = 12;
const DEFAULT_BRIGHTNESS: u8 = 80;
const DEFAULT_PULSE_FREQUENCY_MS: u32 = 5000;
const DEFAULT_RAINBOW_FREQUENCY_MS: u32 = 20000;
const FULL_MOVING_RAINBOW_CYCLE_FREQUENCY_MS: u32 = 10000;

/// Written values for RGB channel and 0 termination.
const LED_RMT_BUF_LEN: usize = 3 * 8 + 1;
const RMT_BUF_LEN: usize = BED_LIGHTSTRIPS_LEDS * LED_RMT_BUF_LEN;
type IrReceiver = Receiver<Nec, DummyPin, time::Instant, remotecontrol::Button<ElegooRemote>>;
type ChannelPayload = remotecontrol::Button<ElegooRemote>;
static IR_CHANNEL: Channel<CriticalSectionRawMutex, ChannelPayload, IR_CMD_CHANNEL_DEPTH> =
    Channel::new();
static IR_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
static IR_RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));
static LED_CHANNEL: Channel<CriticalSectionRawMutex, LedCmd, IR_CMD_CHANNEL_DEPTH> = Channel::new();
static LAST_IR_EVENT: Mutex<Cell<Instant>> = Mutex::new(Cell::new(Instant::ZERO_INSTANT));

#[derive(Debug, Default, Copy, Clone, defmt::Format)]
pub struct ElegooRemote {}

impl RemoteControlModel for ElegooRemote {
    type Cmd = NecCommand;
    const PROTOCOL: infrared::ProtocolId = infrared::ProtocolId::Nec;
    const ADDRESS: u32 = 0;
    const MODEL: &'static str = "Elegoo Remote";

    const BUTTONS: &'static [(u32, Action)] = &[
        (69, Action::Power),
        (70, Action::VolumeUp),
        (71, Action::Stop),
        (68, Action::Left),
        (64, Action::Play),
        (67, Action::Right),
        (7, Action::Down),
        (21, Action::VolumeDown),
        (9, Action::Up),
        (22, Action::Zero),
        (25, Action::Eq),
        (13, Action::Repeat),
        (12, Action::One),
        (24, Action::Two),
        (94, Action::Three),
        (8, Action::Four),
        (28, Action::Five),
        (90, Action::Six),
        (66, Action::Seven),
        (82, Action::Eight),
        (74, Action::Nine),
    ];
}

#[derive(Debug, PartialEq, Eq)]
pub enum LedCmd {
    Switch,
    SelectMode(LightModeCommand),
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!(
        "-- ESP32 Lightstrip Application {} --",
        env!("CARGO_PKG_VERSION")
    );
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_irq_handler);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let led_pin = Output::new(io.pins.gpio0, Level::Low);

    let rmt = Rmt::new_async(peripherals.RMT, 80.MHz()).unwrap();
    let mut rgb_pin = Output::new(io.pins.gpio8, Level::Low);
    rgb_pin.set_low();

    let mut lightstrip_switch = Output::new(io.pins.gpio1, Level::Low);
    lightstrip_switch.set_drive_strength(esp_hal::gpio::DriveStrength::I40mA);
    lightstrip_switch.set_high();

    let mut ir_input = Input::new(io.pins.gpio3, Pull::Up);

    let receiver: IrReceiver = receiver::Builder::default()
        .nec()
        .monotonic::<time::Instant>()
        .frequency(36_000)
        .pin(DummyPin::new_low())
        .remotecontrol(ElegooRemote {})
        .build();

    critical_section::with(|cs| {
        ir_input.listen(gpio::Event::AnyEdge);
        IR_PIN.borrow(cs).replace(Some(ir_input));
        IR_RECEIVER.borrow(cs).replace(Some(receiver));
    });
    let mut ctrl_ticker = Ticker::every(Duration::from_millis(100));
    spawner
        .spawn(led_task(rmt, led_pin, lightstrip_switch))
        .expect("spawning LED task failed");

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(cmd) = IR_CHANNEL.receiver().try_receive() {
            debug!("received NEC command: {:?}", cmd);
            nec_cmd_received = true;
            if let Some(button) = cmd.action() {
                if !cmd.is_repeat() {
                    match button {
                        Action::Power => {
                            // Ignore repeat commands.
                            LED_CHANNEL.sender().try_send(LedCmd::Switch).unwrap()
                        }
                        Action::Zero => LED_CHANNEL
                            .sender()
                            .try_send(LedCmd::SelectMode(LightModeCommand::Rainbow))
                            .unwrap(),
                        Action::One => {
                            LED_CHANNEL
                                .sender()
                                .try_send(LedCmd::SelectMode(LightModeCommand::OneColor))
                                .unwrap();
                        }
                        Action::Two => {
                            LED_CHANNEL
                                .sender()
                                .try_send(LedCmd::SelectMode(LightModeCommand::Pulsing))
                                .unwrap();
                        }
                        Action::Three => {
                            LED_CHANNEL
                                .sender()
                                .try_send(LedCmd::SelectMode(LightModeCommand::MovingRainbow))
                                .unwrap();
                        }
                        _ => (),
                    }
                }
            }
        }
        if !nec_cmd_received {
            ctrl_ticker.next().await;
        }
    }
}

#[handler]
fn gpio_irq_handler() {
    let mut ir_channel_full = false;
    let mut nec_error = None;
    critical_section::with(|cs| {
        let mut ir_pin_borrow = IR_PIN.borrow_ref_mut(cs);
        let ir_pin = ir_pin_borrow.as_mut().unwrap();
        if ir_pin.is_interrupt_set() {
            let current_time = time::now();
            let last_event_cell = LAST_IR_EVENT.borrow(cs);
            let elapsed = current_time - last_event_cell.get();
            last_event_cell.set(current_time);
            match IR_RECEIVER
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .event_edge(elapsed, ir_pin.is_low().unwrap())
            {
                Ok(Some(cmd)) => {
                    if let Err(_e) = IR_CHANNEL.try_send(cmd) {
                        ir_channel_full = true;
                    }
                }
                Ok(None) => (),
                Err(e) => nec_error = Some(e),
            }
        }
        ir_pin.clear_interrupt();
    });
    if ir_channel_full {
        warn!("IR command channel is full");
    }
    if nec_error.is_some() {
        warn!("NEC error: {:?}", nec_error.unwrap());
    }
}

#[derive(Debug, PartialEq, Eq, defmt::Format)]
pub enum LightModeCommand {
    Rainbow,
    OneColor,
    Pulsing,
    MovingRainbow,
}

#[derive(Debug)]
pub enum LightModeState {
    Rainbow(RainbowParameters),
    OneColor(RGB8),
    Pulsing(PulseParameters),
    MovingRainbow(MovingRainbowParameters),
}

#[derive(Debug, Copy, Clone)]
pub struct MovingRainbowParameters {
    current_start_hue: u8,
    current_ticks: u32,
    ms_per_hue_inc: u32,
}
impl Default for LightModeState {
    fn default() -> Self {
        LightModeState::Rainbow(RainbowParameters::new(
            DEFAULT_RAINBOW_FREQUENCY_MS,
            LED_FINE_TICKER_FREQ as u32,
        ))
    }
}

#[derive(Debug, Default)]
pub enum Lightstrip {
    #[default]
    Off,
    On {
        mode: LightModeState,
        brightness: u8,
    },
}

#[derive(Debug)]
pub struct LedState {
    current_brightness: u8,
    current_rainbow_parameters: RainbowParameters,
    current_pulse_parameters: PulseParameters,
    current_one_color: RGB8,
    current_moving_rainbow_parameters: MovingRainbowParameters,
}

impl Default for LedState {
    fn default() -> Self {
        let current_one_color = colors::CRIMSON;
        Self {
            current_brightness: DEFAULT_BRIGHTNESS,
            current_rainbow_parameters: RainbowParameters::new(
                DEFAULT_RAINBOW_FREQUENCY_MS,
                LED_FINE_TICKER_FREQ as u32,
            ),
            current_pulse_parameters: PulseParameters::new(
                current_one_color,
                DEFAULT_PULSE_FREQUENCY_MS,
                LED_FINE_TICKER_FREQ as u32,
            ),
            current_moving_rainbow_parameters: MovingRainbowParameters {
                current_start_hue: 0,
                current_ticks: 0,
                ms_per_hue_inc: FULL_MOVING_RAINBOW_CYCLE_FREQUENCY_MS / 255,
            },
            current_one_color,
        }
    }
}
#[embassy_executor::task]
async fn led_task(
    rmt: Rmt<'static, Async>,
    led_pin: Output<'static>,
    mut switch_pin: Output<'static>,
) {
    let rmt_buffer = [0u32; RMT_BUF_LEN];
    let state = LedState::default();
    let receiver = LED_CHANNEL.receiver();
    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led_helper = SmartLedAdapterAsync::new(rmt.channel0, led_pin, rmt_buffer);
    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(LED_CMD_CHECK_FREQ_MS));
    let mut fine_ticker = Ticker::every(embassy_time::Duration::from_millis(LED_FINE_TICKER_FREQ));
    let divisor = LED_CMD_CHECK_FREQ_MS / LED_FINE_TICKER_FREQ;
    let mut data: [RGB8; BED_LIGHTSTRIPS_LEDS] = [colors::WHITE; BED_LIGHTSTRIPS_LEDS];
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut lightstrip = Lightstrip::Off;
    loop {
        if let Ok(led_cmd) = receiver.try_receive() {
            handle_led_cmd(led_cmd, &mut lightstrip, &state, &mut switch_pin);
        }
        match &mut lightstrip {
            Lightstrip::Off => {
                ticker.next().await;
            }
            Lightstrip::On {
                mode,
                brightness: current_brightness,
            } => {
                match mode {
                    LightModeState::Pulsing(pulse_params) => {
                        data = [pulse_params.color; BED_LIGHTSTRIPS_LEDS];
                        for _ in 0..=divisor {
                            led_helper
                                .write(brightness(
                                    gamma(data.iter().cloned()),
                                    pulse_params.brightness_for_pulse(),
                                ))
                                .await
                                .unwrap();
                            pulse_params.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightModeState::OneColor(rgb) => {
                        data = [*rgb; BED_LIGHTSTRIPS_LEDS];
                        led_helper
                            .write(brightness(gamma(data.iter().cloned()), *current_brightness))
                            .await
                            .unwrap();
                        ticker.next().await;
                    }
                    LightModeState::Rainbow(rainbow_params) => {
                        // Iterate over the rainbow!
                        for _ in 0..=divisor {
                            color.hue = rainbow_params.hue();
                            // Convert from the HSV color space (where we can easily transition from one
                            // color to the other) to the RGB color space that we can then send to the LED
                            data = [hsv2rgb(color); BED_LIGHTSTRIPS_LEDS];
                            // When sending to the LED, we do a gamma correction first (see smart_leds
                            // documentation for details).
                            led_helper
                                .write(brightness(gamma(data.iter().cloned()), *current_brightness))
                                .await
                                .unwrap();
                            rainbow_params.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightModeState::MovingRainbow(params) => {
                        for _ in 0..=divisor {
                            for (idx, next_rgb) in data.iter_mut().enumerate() {
                                if idx == 0 {
                                    color.hue = params.current_start_hue;
                                } else {
                                    color.hue = (params.current_start_hue as u16
                                        + floorf(idx as f32 / BED_LIGHTSTRIPS_LEDS as f32 * 255.0)
                                            as u16
                                            % 255)
                                        as u8;
                                }
                                *next_rgb = hsv2rgb(color);
                            }
                            // Convert from the HSV color space (where we can easily transition from one
                            // color to the other) to the RGB color space that we can then send to the LED
                            led_helper
                                .write(brightness(gamma(data.iter().cloned()), *current_brightness))
                                .await
                                .unwrap();

                            if params.ms_per_hue_inc > LED_FINE_TICKER_FREQ as u32 {
                                // Need to skip increments
                                let skips_needed = roundf(
                                    params.ms_per_hue_inc as f32 / LED_FINE_TICKER_FREQ as f32,
                                ) as u32;
                                // Only update the hue after skipping `skips_needed` ticks
                                if params.current_ticks % skips_needed == 0 {
                                    params.current_start_hue += 1;
                                }
                                params.current_ticks += 1;
                            } else {
                                // Might need to increment more than 1 hue
                                params.current_start_hue +=
                                    ((LED_FINE_TICKER_FREQ as u32 / params.ms_per_hue_inc) % 255)
                                        as u8;
                            }
                            fine_ticker.next().await;
                        }
                    }
                }
            }
        }
    }
}

fn handle_led_cmd(
    led_cmd: LedCmd,
    lightstrip: &mut Lightstrip,
    state: &LedState,
    switch_pin: &mut Output<'static>,
) {
    match led_cmd {
        LedCmd::Switch => match lightstrip {
            Lightstrip::Off => {
                switch_pin.set_high();
                *lightstrip = Lightstrip::On {
                    mode: LightModeState::default(),
                    brightness: DEFAULT_BRIGHTNESS,
                };
                info!("switching lightstrip on");
            }
            Lightstrip::On { .. } => {
                switch_pin.set_low();
                *lightstrip = Lightstrip::Off;
                info!("switching lightstrip off");
            }
        },
        LedCmd::SelectMode(light_mode_cmd) => match light_mode_cmd {
            LightModeCommand::Rainbow => {
                info!("rainbow mode");
                *lightstrip = Lightstrip::On {
                    mode: LightModeState::Rainbow(state.current_rainbow_parameters),
                    brightness: state.current_brightness,
                };
            }
            LightModeCommand::OneColor => {
                info!("one color");
                *lightstrip = Lightstrip::On {
                    mode: LightModeState::OneColor(state.current_one_color),
                    brightness: state.current_brightness,
                };
            }
            LightModeCommand::Pulsing => {
                info!("pulse mode");
                *lightstrip = Lightstrip::On {
                    mode: LightModeState::Pulsing(state.current_pulse_parameters),
                    brightness: state.current_brightness,
                };
            }
            LightModeCommand::MovingRainbow => {
                info!("moving rainbow");
                *lightstrip = Lightstrip::On {
                    mode: LightModeState::MovingRainbow(state.current_moving_rainbow_parameters),
                    brightness: state.current_brightness,
                };
            }
        },
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Scaling {
    current_ticks: u32,
    total_ticks: u32,
}

impl Scaling {
    pub fn new(frequency_ms: u32, fine_ticks_ms: u32) -> Self {
        Self {
            current_ticks: 0,
            total_ticks: Self::total_ticks_for_pulse_frequency(frequency_ms, fine_ticks_ms),
        }
    }

    pub fn update_frequency(&mut self, frequency_ms: u32, fine_ticks_ms: u32) {
        self.total_ticks = Self::total_ticks_for_pulse_frequency(frequency_ms, fine_ticks_ms);
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

#[derive(Debug, Copy, Clone)]
pub struct PulseParameters {
    scaling: Scaling,
    color: RGB8,
}

impl PulseParameters {
    pub fn new(color: RGB8, frequency_ms: u32, fine_ticks_ms: u32) -> Self {
        Self {
            scaling: Scaling::new(frequency_ms, fine_ticks_ms),
            color,
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

#[derive(Debug, Copy, Clone)]
pub struct RainbowParameters {
    scaling: Scaling,
}

impl RainbowParameters {
    pub fn new(frequency_ms: u32, fine_ticks_ms: u32) -> Self {
        Self {
            scaling: Scaling::new(frequency_ms, fine_ticks_ms),
        }
    }

    pub fn hue(&self) -> u8 {
        floorf(self.scaling.current_ticks as f32 / self.scaling.total_ticks as f32 * 255.0) as u8
    }

    pub fn increment(&mut self) {
        self.scaling.increment();
    }
}

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
