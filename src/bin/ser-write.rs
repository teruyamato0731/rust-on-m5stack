use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::{gpio, prelude::*};

use zerocopy::{AsBytes, FromBytes, FromZeroes};

// 状態変数 x, \dot{x}, \theta, \theta
#[derive(Debug, AsBytes, FromBytes, FromZeroes)]
#[repr(C)]
struct State {
    x: f32,
    dx: f32,
    theta: f32,
    dtheta: f32,
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    run().unwrap();
}

fn run() -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;
    let tx = peripherals.pins.gpio1;
    let rx = peripherals.pins.gpio3;

    println!("Starting UART loopback test");
    let config = UartConfig::new().baudrate(Hertz(115_200));
    let uart = UartDriver::new(
        peripherals.uart0,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )?;

    loop {
        // 状態変数を送信
        let s = State {
            x: 1.2,
            dx: 3.4,
            theta: 5.6,
            dtheta: 7.8,
        };

        let buf = s.as_bytes();
        let cobs = cobs_rs::stuff::<16, 18>(buf.try_into().unwrap(), 0);
        let _len = uart.write(&cobs)?;

        FreeRtos::delay_ms(500);
    }
}
