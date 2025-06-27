use core::cell::{Cell, RefCell};

use critical_section::Mutex;
use dummy_pin::DummyPin;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::{
    gpio::Input,
    time::{Instant, Rate},
};
use infrared::{
    protocol::{nec::NecCommand, Nec},
    receiver::DecodingError,
    remotecontrol::{self, Action, RemoteControlModel},
    Receiver,
};

const IR_CMD_CHANNEL_DEPTH: usize = 24;

// #[cfg(feature = "bedroom")]
// pub type Remote = ElegooRemote;
// #[cfg(feature = "tree")]
pub type Remote = BerrybaseRemote;

pub type RemoteButton = remotecontrol::Button<Remote>;

/// 1 us resolution.
pub const IR_HANDLING_FREQ: Rate = Rate::from_hz(1_000_000);
pub type IrReceiver = Receiver<Nec, DummyPin, u64, RemoteButton>;
pub static IR_CHANNEL: Channel<CriticalSectionRawMutex, IrMessage, IR_CMD_CHANNEL_DEPTH> =
    Channel::new();
pub static IR_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
pub static IR_RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));
pub static LAST_IR_EVENT: Mutex<Cell<Instant>> = Mutex::new(Cell::new(Instant::EPOCH));

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
        (64, Action::Play_Pause),
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

#[derive(Debug, Default, Copy, Clone, defmt::Format)]
pub struct BerrybaseRemote {}

impl RemoteControlModel for BerrybaseRemote {
    type Cmd = NecCommand;
    const PROTOCOL: infrared::ProtocolId = infrared::ProtocolId::Nec;
    const ADDRESS: u32 = 0;
    const MODEL: &'static str = "Berrybase Remote";

    const BUTTONS: &'static [(u32, Action)] = &[
        (69, Action::VolumeDown),
        (71, Action::VolumeUp),
        (70, Action::Play_Pause),
        (4, Action::Setup),
        (64, Action::Up),
        (3, Action::Mode),
        (21, Action::Enter),
        (7, Action::Left),
        (9, Action::Right),
        (25, Action::Down),
        (13, Action::Return),
        (22, Action::Plus),
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

#[derive(Debug)]
pub enum IrMessage {
    DecodeError(DecodingError),
    Command(RemoteButton),
}

impl From<DecodingError> for IrMessage {
    fn from(e: DecodingError) -> Self {
        IrMessage::DecodeError(e)
    }
}

impl From<RemoteButton> for IrMessage {
    fn from(e: RemoteButton) -> Self {
        IrMessage::Command(e)
    }
}
