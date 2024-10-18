#![allow(dead_code)]

use crate::{axp192, mcp2515};
use display_interface_spi::SPIInterfaceNoCS;
use esp_idf_hal::{
    gpio::{AnyIOPin, AnyOutputPin, Output, OutputPin, PinDriver},
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver},
    sys::EspError,
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

pub fn m5sc2_init<I2C>(
    axp: &mut axp192::Axp192<I2C>,
    delay: &mut impl embedded_hal::delay::DelayNs,
) -> Result<(), I2C::Error>
where
    I2C: embedded_hal::i2c::ErrorType,
    I2C: embedded_hal::i2c::I2c,
{
    // Default setup for M5Stack Core 2
    axp.set_dcdc1_voltage(3350)?; // Voltage to provide to the microcontroller (this one!)

    axp.set_ldo2_voltage(3300)?; // Peripherals (LCD, ...)
    axp.set_ldo2_on(true)?;

    axp.set_ldo3_voltage(2000)?; // Vibration motor
    axp.set_ldo3_on(false)?;

    axp.set_dcdc3_voltage(2800)?; // LCD backlight
    axp.set_dcdc3_on(true)?;

    axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
    axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                  // expect

    axp.set_gpio2_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
    axp.set_gpio2_output(true)?;

    axp.set_key_mode(
        // Configure how the power button press will work
        axp192::ShutdownDuration::Sd4s,
        axp192::PowerOkDelay::Delay64ms,
        true,
        axp192::LongPress::Lp1000ms,
        axp192::BootTime::Boot512ms,
    )?;

    axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

    axp.set_battery_voltage_adc_enable(true)?;
    axp.set_battery_current_adc_enable(true)?;
    axp.set_acin_current_adc_enable(true)?;
    axp.set_acin_voltage_adc_enable(true)?;

    // Actually reset the LCD
    axp.set_gpio4_output(false)?;
    axp.set_ldo3_on(true)?; // Buzz the vibration motor while intializing ¯\_(ツ)_/¯
    delay.delay_ms(100u32);
    axp.set_gpio4_output(true)?;
    axp.set_ldo3_on(false)?;
    delay.delay_ms(100u32);
    Ok(())
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
