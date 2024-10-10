#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Ticker};
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use esp_hal_embassy::
use embedded_hal::digital::{InputPin, OutputPin};
use esp_backtrace as _;
use esp_hal::time::{self, Instant};
use esp_hal::{
    gpio::{Input, Io, Level, Output, Pull},
    macros::ram,
    peripherals::TIMG0,
    prelude::*,
    rmt::Rmt,
    timer::timg,
};
use esp_hal::{rmt, Blocking};
use esp_println::println;

use esp_hal_smartled::SmartLedsAdapter;
use infrared::protocol::nec::NecCommand;
use infrared::receiver::time::InfraMonotonic;
use infrared::remotecontrol::{self, Action, RemoteControlModel};
use infrared::{protocol::Nec, receiver, Receiver};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite, RGB8,
};

const BED_LIGHTSTRIPS_LEDS: usize = 46;
const IR_CMD_CHANNEL_DEPTH: usize = 12;
const DEFAULT_BRIGHTNESS: u8 = 80;

const RMT_BUF_LEN: usize = BED_LIGHTSTRIPS_LEDS * 24 + 1;
type IrReceiver = Receiver<Nec, DummyPin, time::Instant, remotecontrol::Button<ElegooRemote>>;
type EspTimer = timg::Timer<timg::Timer0<TIMG0>, esp_hal::Blocking>;
type SenderType = Sender<'static, CriticalSectionRawMutex, NecCommand, IR_CMD_CHANNEL_DEPTH>;
type ChannelPayload = remotecontrol::Button<ElegooRemote>;
static IR_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<EspTimer>>> = Mutex::new(RefCell::new(None));
static RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));
static LAST_IR_EVENT: Mutex<Cell<Instant>> = Mutex::new(Cell::new(Instant::ZERO_INSTANT));
//static IR_CMD_SENDER: Mutex<RefCell<Option<SenderType>>> = Mutex::new(RefCell::new(None));
static IR_CHANNEL: Channel<CriticalSectionRawMutex, ChannelPayload, IR_CMD_CHANNEL_DEPTH> =
    Channel::new();
static LED_CHANNEL: Channel<CriticalSectionRawMutex, LedCmd, IR_CMD_CHANNEL_DEPTH> = Channel::new();

#[derive(Debug, Default, Copy, Clone)]
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
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    println!(
        "-- ESP32 Lightstrip Application {} --",
        env!("CARGO_PKG_VERSION")
    );
    let peripherals = esp_hal::init(esp_hal::Config::default());
    //let system = SystemControl::new(peripherals.SYSTEM);
    //let clocks = ClockControl::max(system.clock_control).freeze();

    esp_println::logger::init_logger_from_env();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    //io.set_interrupt_handler(handler);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut led_pin = Output::new(io.pins.gpio0, Level::Low);
    // Hardware timer for high frequency tick counting for IR receiver.
    /*
    let timg1 = timg::TimerGroup::new(peripherals.TIMG1);
    */
    //timg1.timer0.start(1.MHz(), 0xffff_ffff);

    let rmt = Rmt::new(peripherals.RMT, 80.MHz()).unwrap();
    let mut rgb_pin = Output::new(io.pins.gpio8, Level::Low);
    rgb_pin.set_low();

    let mut lightstrip_switch = Output::new(io.pins.gpio1, Level::Low);
    lightstrip_switch.set_drive_strength(esp_hal::gpio::DriveStrength::I40mA);
    lightstrip_switch.set_low();

    let mut ir_input = Input::new(io.pins.gpio3, Pull::Up);

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations

    let receiver: IrReceiver = receiver::Builder::default()
        .nec()
        .monotonic::<time::Instant>()
        .frequency(38_000)
        .pin(DummyPin::new_low())
        .remotecontrol(ElegooRemote {})
        .build();

    let mut ctrl_ticker = Ticker::every(Duration::from_millis(500));
    spawner
        .spawn(ir_receiver_task(ir_input, receiver))
        .expect("spawning IR task failed");
    spawner
        .spawn(led_task(rmt, led_pin, lightstrip_switch))
        .expect("spawning LED task failed");

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(cmd) = IR_CHANNEL.receiver().try_receive() {
            println!("received NEC command: {:?}", cmd);
            if let Some(button) = cmd.action() {
                match button {
                    Action::Power => LED_CHANNEL.sender().try_send(LedCmd::Switch).unwrap(),
                    Action::Zero => (),
                    _ => (),
                }
            }
            nec_cmd_received = true;
        }
        if !nec_cmd_received {
            ctrl_ticker.next().await;
        }
    }
}

#[embassy_executor::task]
async fn ir_receiver_task(mut ir_input: Input<'static>, mut ir_receiver: IrReceiver) {
    let mut last_event = time::Instant::ZERO_INSTANT;
    let sender = IR_CHANNEL.sender();
    loop {
        ir_input.wait_for_any_edge().await;
        let timestamp = time::now();
        let dt = timestamp - last_event;
        last_event = timestamp;
        match ir_receiver.event_edge(dt, ir_input.is_low()) {
            Ok(Some(cmd)) => {
                if let Err(_e) = sender.try_send(cmd) {
                    println!("NEC command channel is full");
                }
            }
            Ok(None) => (),
            Err(e) => {
                println!("NEC error: {:?}", e);
            }
        }
    }
}

#[derive(Debug)]
pub enum GlowMode {
    Rainbow { current_hue: u8 },
}

impl Default for GlowMode {
    fn default() -> Self {
        GlowMode::Rainbow { current_hue: 0 }
    }
}

#[derive(Debug, Default)]
pub enum Lightstrip {
    #[default]
    Off,
    On {
        glow_mode: GlowMode,
        brightness: u8,
    },
}

#[embassy_executor::task]
async fn led_task(
    rmt: Rmt<'static, Blocking>,
    led_pin: Output<'static>,
    mut switch_pin: Output<'static>,
) {
    let rmt_buffer = [0u32; RMT_BUF_LEN];
    let receiver = LED_CHANNEL.receiver();
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);

    let ticker_ms_freq: u64 = 200;
    let fine_ticker_ms_freq: u64 = 20;
    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(ticker_ms_freq));
    let mut fine_ticker = Ticker::every(embassy_time::Duration::from_millis(fine_ticker_ms_freq));
    let divisor = ticker_ms_freq / fine_ticker_ms_freq;
    let mut data: [RGB8; BED_LIGHTSTRIPS_LEDS];
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut lightstrip = Lightstrip::Off;
    loop {
        if let Ok(led_cmd) = receiver.try_receive() {
            if led_cmd == LedCmd::Switch {
                match lightstrip {
                    Lightstrip::Off => {
                        switch_pin.set_high();
                        lightstrip = Lightstrip::On {
                            glow_mode: GlowMode::default(),
                            brightness: DEFAULT_BRIGHTNESS,
                        };
                    }
                    Lightstrip::On { .. } => {
                        switch_pin.set_low();
                        lightstrip = Lightstrip::Off;
                    }
                }
            }
        }
        match &mut lightstrip {
            Lightstrip::Off => {
                ticker.next().await;
            }
            Lightstrip::On {
                glow_mode,
                brightness: current_brightness,
            } => {
                match glow_mode {
                    GlowMode::Rainbow { current_hue } => {
                        // Iterate over the rainbow!
                        for _ in 0..=divisor {
                            color.hue = *current_hue;
                            // Convert from the HSV color space (where we can easily transition from one
                            // color to the other) to the RGB color space that we can then send to the LED
                            data = [hsv2rgb(color); BED_LIGHTSTRIPS_LEDS];
                            // When sending to the LED, we do a gamma correction first (see smart_leds
                            // documentation for details) and then limit the brightness to 10 out of 255 so
                            // that the output it's not too bright.
                            led.write(brightness(gamma(data.iter().cloned()), *current_brightness))
                                .unwrap();
                            *current_hue = current_hue.wrapping_add(1);
                            fine_ticker.next().await;
                        }
                    }
                }
            }
        }
    }
}
