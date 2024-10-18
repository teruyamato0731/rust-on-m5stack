use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal_bus::i2c;
use esp_idf_hal::{
    delay::{FreeRtos, TickType},
    gpio,
    gpio::PinDriver,
    prelude::*,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    uart::{UartConfig, UartDriver},
    units::Hertz,
};
use zerocopy::{AsBytes, FromBytes, FromZeroes};

#[derive(Debug, AsBytes, FromBytes, FromZeroes)]
#[repr(C)]
struct Control {
    u: i16,
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

    let i2c_master = core2::m5_core2::i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        400.kHz().into(),
    )?;

    let i2c_ref_cell = core::cell::RefCell::new(i2c_master);

    core2::power::Power::new(i2c::RefCellDevice::new(&i2c_ref_cell))
        .init(&mut FreeRtos)
        .unwrap();

    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio18;
    let serial_in = peripherals.pins.gpio38; // SDI
    let serial_out = peripherals.pins.gpio23; // SDO
    let cs_1 = peripherals.pins.gpio5;

    let driver = SpiDriver::new(
        spi,
        sclk,
        serial_out,
        Some(serial_in),
        &SpiDriverConfig::new(),
    )?;

    let config = SpiConfig::new().baudrate(20.MHz().into());
    let lcd_spi_master = SpiDeviceDriver::new(&driver, Some(cs_1), &config)?;

    let dc = PinDriver::output(peripherals.pins.gpio15)?;
    let spi_iface = SPIInterfaceNoCS::new(lcd_spi_master, dc);
    let mut display = mipidsi::Builder::ili9342c_rgb565(spi_iface)
        .with_display_size(320, 240)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(
            &mut esp_idf_hal::delay::FreeRtos,
            None::<PinDriver<esp_idf_hal::gpio::AnyOutputPin, esp_idf_hal::gpio::Output>>,
        )
        .unwrap();

    loop {
        // UARTを読み取り
        let mut buf = [0u8; 4];
        let timeout = TickType::new_millis(1000);
        let len = uart.read(&mut buf, timeout.into())?;

        println!("Received: {:?}, len: {}", buf, len);

        // 4バイトのデータを受信したら、制御入力として解釈
        let control = if len == 4 {
            let (buf, _): ([u8; 2], usize) = cobs_rs::unstuff(buf, 0);
            Control::read_from(&buf).unwrap()
        } else {
            Control { u: 0 }
        };

        // Make the display all green
        display.clear(Rgb565::BLACK).unwrap();
        // Draw with embedded_graphics
        Text::with_alignment(
            &format!("control: {}", control.u),
            Point::new(160, 120),
            MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::WHITE),
            embedded_graphics::text::Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        FreeRtos::delay_ms(1000);
    }
}
