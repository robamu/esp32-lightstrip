#![no_std]
#![no_main]

use defmt::{debug, println, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp32_lightstrip::handle_nec_cmd;
use esp32_lightstrip::ir;
use esp32_lightstrip::led::led_task;
use esp_backtrace as _;
use esp_hal::gpio;
use esp_hal::gpio::OutputConfig;
use esp_hal::rmt::{self, Rmt, RxChannelCreatorAsync};
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::{
    gpio::{Level, Output},
    timer::timg,
};
use esp_println as _;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // esp_println::defmt::();
    println!(
        "-- ESP32 Lightstrip Application v{} --",
        env!("CARGO_PKG_VERSION")
    );
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let rng = Rng::new(peripherals.RNG);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let led_pin = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());

    let mut rgb_pin = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    rgb_pin.set_low();

    let switch_cfg = OutputConfig::default().with_drive_strength(gpio::DriveStrength::_5mA);
    let mut lightstrip_switch = Output::new(peripherals.GPIO1, Level::Low, switch_cfg);
    lightstrip_switch.set_low();

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap()
        .into_async();

    let ir_rx_config = rmt::RxChannelConfig::default()
        .with_clk_divider(255)
        .with_idle_threshold(10000);
    let ir_rx_channel = rmt
        .channel2
        .configure(peripherals.GPIO2, ir_rx_config)
        .unwrap();

    let ir_receiver: ir::IrReceiver = infrared::Receiver::new(80_000_000);
    let ir_channel = ir::IR_CHANNEL.take();
    let ir_msg_sender = ir_channel.sender();
    let ir_msg_receiver = ir_channel.receiver();

    let mut ctrl_ticker = Ticker::every(Duration::from_millis(100));
    spawner
        .spawn(led_task(rmt.channel0, rng, led_pin, lightstrip_switch))
        .expect("spawning LED task failed");
    spawner
        .spawn(ir::ir_receive(ir_rx_channel, ir_receiver, ir_msg_sender))
        .expect("spawning IR task failed");

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(msg) = ir_msg_receiver.try_receive() {
            match msg {
                ir::IrMessage::DecodeError(decoding_error) => {
                    warn!("IR decoding error: {:?}", decoding_error);
                }
                ir::IrMessage::Command(cmd) => {
                    debug!("received NEC command: {:?}", cmd);
                    nec_cmd_received = true;
                    if let Some(button) = cmd.action() {
                        handle_nec_cmd(button, cmd.is_repeat());
                    }
                }
            }
        }
        if !nec_cmd_received {
            ctrl_ticker.next().await;
        }
    }
}
