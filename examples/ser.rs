use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::TickType;
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::{gpio, prelude::*};
use std::fmt::Write;

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
    let mut uart = UartDriver::new(
        peripherals.uart0,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )?;

    loop {
        // UARTを読み取り
        let mut buf = [0u8; 32];
        let timeout = TickType::new_millis(10);
        let len = uart.read(&mut buf, timeout.into())?;

        if len > 0 {
            // 受信したデータをUARTに書き込む
            let _len = uart.write(&buf[..len])?;
        } else {
            // 受信データがない場合は、UARTにメッセージを書き込む
            writeln!(uart, "Hello, World!")?;
        }

        FreeRtos::delay_ms(500);
    }
}
