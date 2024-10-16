#![no_std]
#![no_main]
use defmt::warn;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp32_lightstrip::gpio_irq_handler;
use esp32_lightstrip::handle_nec_cmd;
use esp32_lightstrip::ir::IrMessage;
use esp32_lightstrip::ir::IrReceiver;
use esp32_lightstrip::ir::Remote;
use esp32_lightstrip::ir::IR_CHANNEL;
use esp32_lightstrip::ir::IR_PIN;
use esp32_lightstrip::ir::IR_RECEIVER;
use esp_backtrace as _;
use esp_hal::gpio;
use esp_hal::time;
use esp_hal::{
    gpio::{Input, Io, Pull},
    prelude::*,
    timer::timg,
};
use esp_println as _;
use infrared::receiver;
use log::info;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_irq_handler);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut ir_input = Input::new(io.pins.gpio2, Pull::Up);

    let receiver: IrReceiver = receiver::Builder::default()
        .nec()
        .monotonic::<time::Instant>()
        .frequency(38_000)
        .pin(DummyPin::new_low())
        .remotecontrol(Remote {})
        .build();

    critical_section::with(|cs| {
        ir_input.listen(gpio::Event::AnyEdge);
        IR_PIN.borrow(cs).replace(Some(ir_input));
        IR_RECEIVER.borrow(cs).replace(Some(receiver));
    });
    let mut ctrl_ticker = Ticker::every(Duration::from_millis(100));

    let mut nec_cmd_received;
    loop {
        nec_cmd_received = false;
        if let Ok(msg) = IR_CHANNEL.receiver().try_receive() {
            match msg {
                IrMessage::DecodeError(decoding_error) => {
                    warn!("IR decoding error: {:?}", decoding_error);
                }
                IrMessage::Command(cmd) => {
                    info!("received NEC command: {:?}", cmd);
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
