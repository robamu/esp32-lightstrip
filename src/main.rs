#![no_std]
#![no_main]

use defmt::{println, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp32_lightstrip::gpio_irq_handler;
use esp32_lightstrip::handle_nec_cmd;
use esp32_lightstrip::ir;
use esp32_lightstrip::led::led_task;
use esp_backtrace as _;
use esp_hal::gpio;
use esp_hal::gpio::InputConfig;
use esp_hal::gpio::OutputConfig;
use esp_hal::rmt::Rmt;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::{
    gpio::{Input, Io, Level, Output, Pull},
    timer::timg,
};
use infrared::receiver;
use log::debug;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    println!(
        "-- ESP32 Lightstrip Application v{} --",
        env!("CARGO_PKG_VERSION")
    );
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let rng = Rng::new(peripherals.RNG);

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_irq_handler);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let led_pin = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap()
        .into_async();
    let mut rgb_pin = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    rgb_pin.set_low();

    let switch_cfg = OutputConfig::default().with_drive_strength(gpio::DriveStrength::_5mA);
    let mut lightstrip_switch = Output::new(peripherals.GPIO1, Level::Low, switch_cfg);
    lightstrip_switch.set_low();

    let input_cfg = InputConfig::default().with_pull(Pull::Up);
    let mut ir_input = Input::new(peripherals.GPIO2, input_cfg);

    let receiver: ir::IrReceiver = receiver::Builder::default()
        .nec()
        .monotonic::<u64>()
        .frequency(ir::IR_HANDLING_FREQ.as_hz())
        .remotecontrol(ir::Remote {})
        .build();

    critical_section::with(|cs| {
        ir_input.listen(gpio::Event::AnyEdge);
        ir::IR_PIN.borrow(cs).replace(Some(ir_input));
        ir::IR_RECEIVER.borrow(cs).replace(Some(receiver));
    });
    let mut ctrl_ticker = Ticker::every(Duration::from_millis(100));
    spawner
        .spawn(led_task(rmt, rng, led_pin, lightstrip_switch))
        .expect("spawning LED task failed");

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(msg) = ir::IR_CHANNEL.receiver().try_receive() {
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
