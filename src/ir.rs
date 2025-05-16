use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::{
    gpio,
    rmt::{self, PulseCode, RxChannelAsync as _},
    Async,
};
use infrared::{
    protocol::{nec::NecCommand, Nec},
    receiver::{DecodingError, NoPin},
    remotecontrol::{self, Action, RemoteControlModel},
    Receiver,
};

#[cfg(feature = "bedroom")]
pub type Remote = ElegooRemote;
#[cfg(feature = "tree")]
pub type Remote = BerrybaseRemote;

const IR_CMD_CHANNEL_DEPTH: usize = 24;

pub type RemoteButton = remotecontrol::Button<Remote>;
pub type IrChannelType =
    embassy_sync::channel::Channel<NoopRawMutex, IrMessage, IR_CMD_CHANNEL_DEPTH>;
pub static IR_CHANNEL: static_cell::ConstStaticCell<IrChannelType> =
    static_cell::ConstStaticCell::new(IrChannelType::new());

pub type IrReceiver = Receiver<Nec, NoPin, u32, RemoteButton>;

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

#[embassy_executor::task]
pub async fn ir_receive(
    mut channel: rmt::Channel<Async, 2>,
    mut ir_receiver: IrReceiver,
    ir_msg_tx: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        IrMessage,
        IR_CMD_CHANNEL_DEPTH,
    >,
) {
    let mut data = [u32::new(gpio::Level::High, 1, gpio::Level::Low, 1); 48];

    loop {
        channel.receive(&mut data).await.unwrap();

        for entry in data {
            if entry.length1() == 0 {
                break;
            }

            let res = ir_receiver.event(
                entry.length1() as u32 * 255,
                entry.level1() == gpio::Level::High,
            );

            if let Err(e) = res {
                ir_msg_tx.try_send(e.into()).unwrap();
                break;
            }

            if let Some(cmd) = res.unwrap() {
                // execute command
                defmt::debug!("IR CMD: {:?}", cmd);
                ir_msg_tx.try_send(cmd.into()).unwrap();
                break;
            }

            if entry.length2() == 0 {
                break;
            }

            let res = ir_receiver.event(
                entry.length2() as u32 * 255,
                entry.level2() == gpio::Level::High,
            );

            if let Err(e) = res {
                ir_msg_tx.try_send(e.into()).unwrap();
                break;
            }

            if let Some(cmd) = res.unwrap() {
                // execute command
                defmt::debug!("IR CMD: {:?}", cmd);
                ir_msg_tx.try_send(cmd.into()).unwrap();
                break;
            }
        }
    }
}
