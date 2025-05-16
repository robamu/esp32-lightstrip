#![no_std]

use infrared::remotecontrol::Action;
use led::{LedCmd, LightMode, LED_CHANNEL};
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
        defmt::warn!("no LED command for action {:?}", action);
        return;
    }
    let led_cmd = led_cmd_opt.unwrap();
    LED_CHANNEL.sender().try_send(led_cmd).unwrap();
}
