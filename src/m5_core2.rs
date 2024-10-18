#![allow(dead_code)]

use crate::mcp2515;
use display_interface_spi::SPIInterfaceNoCS;
use esp_idf_hal::{
    gpio::{AnyIOPin, AnyOutputPin, InputPin, Output, OutputPin, PinDriver},
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver},
    sys::EspError,
    uart::{Uart, UartConfig, UartDriver},
    units::Hertz,
};

pub fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> Result<I2cDriver<'d>, EspError> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

type SpiInterface<'a> =
    SPIInterfaceNoCS<SpiDeviceDriver<'a, &'a SpiDriver<'a>>, PinDriver<'a, AnyOutputPin, Output>>;

type DisplayType<'a> = mipidsi::Display<
    SpiInterface<'a>,
    mipidsi::models::ILI9342CRgb565,
    PinDriver<'a, AnyOutputPin, Output>,
>;

pub fn initialize_display<'a>(
    driver: &'a SpiDriver<'a>,
    config: &SpiConfig,
    cs: AnyOutputPin,
    dc: AnyOutputPin,
) -> DisplayType<'a> {
    let lcd_spi_master =
        SpiDeviceDriver::new(driver, Some(cs), config).expect("Failed to initialize SPI device");

    let dc = PinDriver::output(dc).expect("Failed to initialize DC pin");
    let spi_iface = SPIInterfaceNoCS::new(lcd_spi_master, dc);

    mipidsi::Builder::ili9342c_rgb565(spi_iface)
        .with_display_size(320, 240)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(
            &mut esp_idf_hal::delay::FreeRtos,
            None::<PinDriver<AnyOutputPin, Output>>,
        )
        .expect("Failed to initialize display")
}

pub fn initialize_can<'a>(
    driver: &'a SpiDriver<'a>,
    cs_pin: impl OutputPin,
    config: &'a SpiConfig,
) -> mcp2515::MCP2515<SpiDeviceDriver<'a, &'a SpiDriver<'a>>> {
    let can_spi_master = SpiDeviceDriver::new(driver, Some(cs_pin), config)
        .expect("Failed to create SPI device driver");
    let mut can = mcp2515::MCP2515::new(can_spi_master);
    can.init(&mut esp_idf_hal::delay::FreeRtos)
        .expect("Failed to initialize MCP2515");
    can.set_mode(mcp2515::Mode::Normal, &mut esp_idf_hal::delay::FreeRtos)
        .expect("Failed to set mode");
    can
}

pub fn initialize_uart<'d>(
    uart: impl Peripheral<P = impl Uart> + 'd,
    tx: impl Peripheral<P = impl OutputPin> + 'd,
    rx: impl Peripheral<P = impl InputPin> + 'd,
) -> Result<UartDriver<'d>, EspError> {
    let uart_config = UartConfig::new().baudrate(Hertz(115_200));
    let uart = UartDriver::new(
        uart,
        tx,
        rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart_config,
    )?;
    Ok(uart)
}
