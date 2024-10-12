use core::cell::{Cell, RefCell};

use critical_section::Mutex;
use dummy_pin::DummyPin;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::{
    gpio::Input,
    time::{self, Instant},
};
use infrared::{
    protocol::{nec::NecCommand, Nec},
    receiver::{time::InfraMonotonic, DecodingError},
    remotecontrol::{self, Action, RemoteControlModel},
    Receiver,
};

const IR_CMD_CHANNEL_DEPTH: usize = 24;

pub type IrReceiver = Receiver<Nec, DummyPin, time::Instant, remotecontrol::Button<ElegooRemote>>;
pub static IR_CHANNEL: Channel<CriticalSectionRawMutex, IrMessage, IR_CMD_CHANNEL_DEPTH> =
    Channel::new();
pub static IR_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
pub static IR_RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));
pub static LAST_IR_EVENT: Mutex<Cell<Instant>> = Mutex::new(Cell::new(Instant::ZERO_INSTANT));

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
#[derive(Debug)]
pub enum IrMessage {
    DecodeError(DecodingError),
    Command(remotecontrol::Button<ElegooRemote>),
}

impl From<DecodingError> for IrMessage {
    fn from(e: DecodingError) -> Self {
        IrMessage::DecodeError(e)
    }
}

impl From<remotecontrol::Button<ElegooRemote>> for IrMessage {
    fn from(e: remotecontrol::Button<ElegooRemote, NecCommand>) -> Self {
        IrMessage::Command(e)
    }
}
