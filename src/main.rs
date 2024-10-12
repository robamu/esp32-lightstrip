#![no_std]
#![no_main]
mod led;

use core::cell::Cell;
use core::cell::RefCell;

use critical_section::Mutex;
use defmt::println;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::InputPin;
use esp_backtrace as _;
use esp_hal::gpio;
use esp_hal::rmt::Rmt;
use esp_hal::rng::Rng;
use esp_hal::time;
use esp_hal::time::Instant;
use esp_hal::Async;
use esp_hal::{
    gpio::{Input, Io, Level, Output, Pull},
    prelude::*,
    timer::timg,
};
use esp_println as _;
use infrared::protocol::nec::NecCommand;
use infrared::receiver::time::InfraMonotonic;
use infrared::remotecontrol::{self, Action, RemoteControlModel};
use infrared::{protocol::Nec, receiver, Receiver};
use led::AdjustableSpeed;
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
    SelectMode(LightMode),
    IncreaseBrightness,
    DecreaseBrightness,
    IncreaseSpeed,
    DecreaseSpeed,
    NextColor,
    PrevColor,
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!(
        "-- ESP32 Lightstrip Application {} --",
        env!("CARGO_PKG_VERSION")
    );
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let rng = Rng::new(peripherals.RNG);

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
        .spawn(led_task(rmt, rng, led_pin, lightstrip_switch))
        .expect("spawning LED task failed");

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(cmd) = IR_CHANNEL.receiver().try_receive() {
            debug!("received NEC command: {:?}", cmd);
            nec_cmd_received = true;
            if let Some(button) = cmd.action() {
                handle_nec_cmd(button, cmd.is_repeat());
            }
        }
        if !nec_cmd_received {
            ctrl_ticker.next().await;
        }
    }
}

fn handle_nec_cmd(action: Action, is_repeat: bool) {
    match action {
        Action::Power
        | Action::One
        | Action::Two
        | Action::Three
        | Action::Four
        | Action::Five
        | Action::Six
        | Action::Seven
        | Action::Eight
        | Action::Nine
        | Action::Left
        | Action::Right => {
            if is_repeat {
                return;
            }
        }
        _ => (),
    }
    match action {
        Action::Power => LED_CHANNEL.sender().try_send(LedCmd::Switch).unwrap(),
        Action::Zero => LED_CHANNEL
            .sender()
            .try_send(LedCmd::SelectMode(LightMode::OneColor))
            .unwrap(),
        Action::One => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::SelectMode(LightMode::Pulsing))
                .unwrap();
        }
        Action::Two => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::SelectMode(LightMode::Rainbow))
                .unwrap();
        }
        Action::Three => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::SelectMode(LightMode::MovingRainbow))
                .unwrap();
        }
        Action::Four => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::SelectMode(LightMode::Disco))
                .unwrap();
        }
        Action::VolumeUp => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::IncreaseBrightness)
                .unwrap();
        }
        Action::VolumeDown => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::DecreaseBrightness)
                .unwrap();
        }
        Action::Up => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::IncreaseSpeed)
                .unwrap();
        }
        Action::Down => {
            LED_CHANNEL
                .sender()
                .try_send(LedCmd::DecreaseSpeed)
                .unwrap();
        }
        Action::Left => {
            LED_CHANNEL.sender().try_send(LedCmd::PrevColor).unwrap();
        }
        Action::Right => {
            LED_CHANNEL.sender().try_send(LedCmd::NextColor).unwrap();
        }
        _ => (),
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
            ir_pin.clear_interrupt();
        }
    });
    if ir_channel_full {
        warn!("IR command channel is full");
    }
    if nec_error.is_some() {
        warn!("NEC error: {:?}", nec_error.unwrap());
    }
}

#[derive(Debug, Default, PartialEq, Eq, defmt::Format)]
pub enum LightMode {
    #[default]
    Rainbow,
    OneColor,
    Pulsing,
    MovingRainbow,
    Disco,
}

#[derive(Debug, Default)]
pub enum Mode {
    #[default]
    Off,
    On(LightMode),
}

pub struct Ledstrip {
    mode: Mode,
    led_was_switched_in_cycle: bool,
    color_was_changed_in_cycle: bool,
    brightness: u8,
    color_index: usize,
    rainbow_parameters: led::rainbow::Params,
    pulse_parameters: led::pulse::Params,
    moving_rainbow_parameters: led::moving_rainbow::Params,
    disco_params: led::disco::Params,
}

impl Default for Ledstrip {
    fn default() -> Self {
        Self {
            mode: Mode::Off,
            color_index: 0,
            led_was_switched_in_cycle: false,
            color_was_changed_in_cycle: false,
            brightness: DEFAULT_BRIGHTNESS,
            rainbow_parameters: led::rainbow::Params::new(
                led::rainbow::DEFAULT_FREQUENCY_MS,
                LED_FINE_TICKER_FREQ as u32,
            ),
            pulse_parameters: led::pulse::Params::new(led::pulse::DEFAULT_FREQUENCY_MS),
            moving_rainbow_parameters: led::moving_rainbow::Params::default(),
            disco_params: Default::default(),
        }
    }
}

impl Ledstrip {
    fn all_commands_were_handled(&mut self) {
        self.led_was_switched_in_cycle = false;
        self.color_was_changed_in_cycle = false;
    }

    fn handle_led_cmd(&mut self, led_cmd: LedCmd, switch_pin: &mut Output<'static>) {
        match led_cmd {
            LedCmd::Switch => match self.mode {
                Mode::Off => {
                    // We only allow one switch per cycle.
                    if !self.led_was_switched_in_cycle {
                        switch_pin.set_high();
                        self.mode = Mode::On(LightMode::default());
                        self.led_was_switched_in_cycle = true;
                        info!("switching lightstrip on");
                    }
                }
                Mode::On { .. } => {
                    if !self.led_was_switched_in_cycle {
                        switch_pin.set_low();
                        self.mode = Mode::Off;
                        self.led_was_switched_in_cycle = true;
                        info!("switching lightstrip off");
                    }
                }
            },
            LedCmd::SelectMode(light_mode_cmd) => match light_mode_cmd {
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
            },
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
                if self.color_was_changed_in_cycle {
                    return;
                }
                if self.color_index == 0 {
                    self.color_index = COLOR_SET.len() - 1;
                } else {
                    self.color_index -= 1;
                }
                self.color_was_changed_in_cycle = true;
            }
            LedCmd::NextColor => {
                if self.color_was_changed_in_cycle {
                    return;
                }
                if self.color_index == COLOR_SET.len() - 1 {
                    self.color_index = 0;
                } else {
                    self.color_index += 1;
                }
                self.color_was_changed_in_cycle = true;
            }
        }
    }
}

#[embassy_executor::task]
async fn led_task(
    rmt: Rmt<'static, Async>,
    mut rng: Rng,
    led_pin: Output<'static>,
    mut switch_pin: Output<'static>,
) {
    let rmt_buffer = [0u32; esp_hal_smartled::asynch::buffer_size(BED_LIGHTSTRIPS_LEDS)];
    let mut ledstrip = Ledstrip::default();
    let receiver = LED_CHANNEL.receiver();
    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led_helper =
        esp_hal_smartled::asynch::SmartLedAdapterAsync::new(rmt.channel0, led_pin, rmt_buffer);
    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(LED_CMD_CHECK_FREQ_MS));
    let mut fine_ticker = Ticker::every(embassy_time::Duration::from_millis(LED_FINE_TICKER_FREQ));
    let divisor = LED_CMD_CHECK_FREQ_MS / LED_FINE_TICKER_FREQ;
    let mut data: [RGB8; BED_LIGHTSTRIPS_LEDS] = [colors::WHITE; BED_LIGHTSTRIPS_LEDS];
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    loop {
        while let Ok(led_cmd) = receiver.try_receive() {
            ledstrip.handle_led_cmd(led_cmd, &mut switch_pin);
        }
        ledstrip.all_commands_were_handled();
        match &mut ledstrip.mode {
            Mode::Off => {
                ticker.next().await;
            }
            Mode::On(mode) => {
                match mode {
                    LightMode::Pulsing => {
                        data = [COLOR_SET[ledstrip.color_index]; BED_LIGHTSTRIPS_LEDS];
                        for _ in 0..=divisor {
                            led_helper
                                .write(brightness(
                                    gamma(data.iter().cloned()),
                                    ledstrip.pulse_parameters.brightness_for_pulse(),
                                ))
                                .await
                                .unwrap();
                            ledstrip.pulse_parameters.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightMode::OneColor => {
                        data = [COLOR_SET[ledstrip.color_index]; BED_LIGHTSTRIPS_LEDS];
                        led_helper
                            .write(brightness(gamma(data.iter().cloned()), ledstrip.brightness))
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
                            data = [hsv2rgb(color); BED_LIGHTSTRIPS_LEDS];
                            // When sending to the LED, we do a gamma correction first (see smart_leds
                            // documentation for details).
                            led_helper
                                .write(brightness(gamma(data.iter().cloned()), ledstrip.brightness))
                                .await
                                .unwrap();
                            ledstrip.rainbow_parameters.increment();
                            fine_ticker.next().await;
                        }
                    }
                    LightMode::Disco => {
                        let now = time::now();
                        if (now - ledstrip.disco_params.last_randomization).to_millis()
                            > ledstrip.disco_params.frequency_ms.current as u64
                        {
                            for next_rgb in data.iter_mut() {
                                color.hue =
                                    roundf((rng.random() as f32 / u32::MAX as f32) * 255.0) as u8;
                                *next_rgb = hsv2rgb(color);
                            }
                            led_helper
                                .write(brightness(gamma(data.iter().cloned()), ledstrip.brightness))
                                .await
                                .unwrap();
                            ledstrip.disco_params.last_randomization = now;
                        }
                        fine_ticker.next().await;
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
                                .write(brightness(gamma(data.iter().cloned()), ledstrip.brightness))
                                .await
                                .unwrap();

                            if ledstrip.moving_rainbow_parameters.ms_per_hue_inc
                                > LED_FINE_TICKER_FREQ as u32
                            {
                                let now = time::now();
                                if (now - ledstrip.moving_rainbow_parameters.last_hue_increment)
                                    .to_millis()
                                    > ledstrip.moving_rainbow_parameters.ms_per_hue_inc as u64
                                {
                                    ledstrip.moving_rainbow_parameters.current_start_hue += 1;
                                    ledstrip.moving_rainbow_parameters.last_hue_increment = now;
                                }
                            } else {
                                // Might need to increment more than 1 hue
                                ledstrip.moving_rainbow_parameters.current_start_hue +=
                                    ((LED_FINE_TICKER_FREQ as u32
                                        / ledstrip.moving_rainbow_parameters.ms_per_hue_inc)
                                        % 255) as u8;
                            }
                            fine_ticker.next().await;
                        }
                    }
                }
            }
        }
    }
}
