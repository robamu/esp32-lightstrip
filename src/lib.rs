#![no_std]

use esp_hal::{handler, time::Instant};
use infrared::remotecontrol::Action;
use ir::{IR_CHANNEL, IR_PIN, IR_RECEIVER, LAST_IR_EVENT};
use led::{LedCmd, LightMode, LED_CHANNEL};
use log::{error, warn};
pub mod conf;
pub mod ir;
pub mod led;

#[cfg(not(any(feature = "bedroom", feature = "tree")))]
compile_error!(
    "at least one target feature must be active. Targets:
    - bedroom
    - tree
"
);

pub fn repeats_should_be_ignored(action: Action) -> bool {
    matches!(
        action,
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
            | Action::Right
    )
}

pub fn action_to_led_cmd(action: Action) -> Option<LedCmd> {
    match action {
        Action::Power => Some(LedCmd::Switch),
        #[cfg(feature = "tree")]
        Action::Play_Pause => Some(LedCmd::Switch),
        Action::Zero => Some(LedCmd::SelectMode(LightMode::OneColor)),
        Action::One => Some(LedCmd::SelectMode(LightMode::Pulsing)),
        Action::Two => Some(LedCmd::SelectMode(LightMode::Rainbow)),
        Action::Three => Some(LedCmd::SelectMode(LightMode::MovingRainbow)),
        Action::Four => Some(LedCmd::SelectMode(LightMode::Disco)),
        Action::VolumeUp => Some(LedCmd::IncreaseBrightness),
        Action::VolumeDown => Some(LedCmd::DecreaseBrightness),
        Action::Up => Some(LedCmd::IncreaseSpeed),
        Action::Down => Some(LedCmd::DecreaseSpeed),
        Action::Left => Some(LedCmd::PrevColor),
        Action::Right => Some(LedCmd::NextColor),
        _ => None,
    }
}

pub fn handle_nec_cmd(action: Action, is_repeat: bool) {
    if is_repeat && repeats_should_be_ignored(action) {
        return;
    }
    let led_cmd_opt = action_to_led_cmd(action);
    if led_cmd_opt.is_none() {
        warn!("no LED command for action {:?}", action);
        return;
    }
    let led_cmd = led_cmd_opt.unwrap();
    LED_CHANNEL.sender().try_send(led_cmd).unwrap();
}

#[handler]
pub fn gpio_irq_handler() {
    let mut ir_channel_full = false;
    let mut nec_error = None;
    let opt_cmd = critical_section::with(|cs| {
        let mut ir_pin_borrow = IR_PIN.borrow_ref_mut(cs);
        let ir_pin = ir_pin_borrow.as_mut().unwrap();
        if ir_pin.is_interrupt_set() {
            ir_pin.clear_interrupt();
            let current_time = Instant::now();
            let last_event_cell = LAST_IR_EVENT.borrow(cs);
            let elapsed = current_time - last_event_cell.get();
            last_event_cell.set(current_time);
            let cmd = match IR_RECEIVER
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .event_edge(elapsed.as_micros(), ir_pin.is_low())
            {
                Ok(opt_cmd) => opt_cmd,
                Err(e) => {
                    nec_error = Some(e);
                    None
                }
            };
            return cmd;
        }
        None
    });

    if let Some(cmd) = opt_cmd {
        if let Err(_e) = IR_CHANNEL.try_send(cmd.into()) {
            ir_channel_full = true;
        }
    }
    if let Some(nec_error) = nec_error {
        IR_CHANNEL.sender().try_send(nec_error.into()).ok();
    }
    if ir_channel_full {
        error!("IR command channel is full");
    }
}
