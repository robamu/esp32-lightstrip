#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::TIMG0,
    prelude::*,
    rmt::Rmt,
    timer::timg,
};

use esp_hal_smartled::SmartLedsAdapter;
//use embassy_time::Timer;
use infrared::{protocol::Nec, receiver, Receiver};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite, RGB8,
};

const BED_LIGHTSTRIPS_LEDS: usize = 46;

type IrReceiver = Receiver<Nec, DummyPin>;
type Timer = timg::Timer<timg::Timer0<TIMG0>, esp_hal::Blocking>;
static IR_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
static RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    //let system = SystemControl::new(peripherals.SYSTEM);
    //let clocks = ClockControl::max(system.clock_control).freeze();

    esp_println::logger::init_logger_from_env();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

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

    let mut ir_input = Input::new(io.pins.gpio2, Pull::None);
    ir_input.listen(esp_hal::gpio::Event::AnyEdge);

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let rmt_buffer = [0u32; BED_LIGHTSTRIPS_LEDS * 24 + 1];

    let mut led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio0, rmt_buffer);

    let receiver = receiver::Builder::default()
        .nec()
        .frequency(38_000)
        .pin(DummyPin::new_low())
        .build();

    critical_section::with(|cs| {
        IR_PIN.borrow(cs).replace(Some(ir_input));
        RECEIVER.borrow(cs).replace(Some(receiver));
    });

    //let delay = Delay::new(&clocks);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data: [RGB8; BED_LIGHTSTRIPS_LEDS];
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut led_ticker = Ticker::every(Duration::from_millis(20));

    loop {
        // Iterate over the rainbow!
        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = [hsv2rgb(color); BED_LIGHTSTRIPS_LEDS];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 80))
                .unwrap();
            led_ticker.next().await;
        }
        ticker.next().await;
    }
}

#[handler]
fn handler() {
    critical_section::with(|cs| {
        if IR_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
        {
            /*
            match RECEIVER
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .event_edge(t, true)
            {
                Ok(opt_cmd) => {}
                Err(_) => {}
            }
            */
        }
    });
}
