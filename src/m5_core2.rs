#![allow(dead_code)]

use crate::axp192;
use display_interface_spi::SPIInterfaceNoCS;
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{AnyIOPin, PinDriver},
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver},
    sys::EspError,
    units::Hertz,
};

const SLAVE_ADDR: u8 = 0x68;
const CONFIG: u8 = 0x1A;
// const GYRO_CONFIG: u8 = 0x1B;
// const ACCEL_CONFIG: u8 = 0x1C;
const WHOAMI: u8 = 0x75;
const PWR_MGMT_1: u8 = 0x6B;
const ACCEL_XOUT_H: u8 = 0x3B;
const GYRO_XOUT_H: u8 = 0x43;

pub fn imu_init<I2C>(i2c_master: &mut I2C) -> Result<(), I2C::Error>
where
    I2C: embedded_hal::i2c::I2c,
{
    i2c_master.write(SLAVE_ADDR, &[WHOAMI])?;
    let mut tmp = [0];
    i2c_master.read(SLAVE_ADDR, &mut tmp)?;
    log::debug!("WHOAMI: {:#X}", tmp[0]);
    assert_eq!(tmp[0], 0x19);

    // reset
    i2c_master.write(SLAVE_ADDR, &[PWR_MGMT_1, 0x80])?;
    // wait for reset
    FreeRtos::delay_ms(15);
    // activate gyro and accel
    i2c_master.write(SLAVE_ADDR, &[PWR_MGMT_1, 0x01])?;
    // set gyro range to +-500 deg/s and accel range to +-4g
    i2c_master.write(SLAVE_ADDR, &[CONFIG, 0x1A, 0x08, 0x08])?;

    Ok(())
}

pub fn imu_read_accel<I2C>(i2c_master: &mut I2C) -> Result<(f32, f32, f32), I2C::Error>
where
    I2C: embedded_hal::i2c::I2c,
{
    i2c_master.write(SLAVE_ADDR, &[ACCEL_XOUT_H])?;
    let mut buffer = [0; 6];
    i2c_master.read(SLAVE_ADDR, &mut buffer)?;
    Ok((
        conv((buffer[0] as i16) << 8 | buffer[1] as i16, 4.0 * 9.8),
        conv((buffer[2] as i16) << 8 | buffer[3] as i16, 4.0 * 9.8),
        conv((buffer[4] as i16) << 8 | buffer[5] as i16, 4.0 * 9.8),
    ))
}

pub fn imu_read_gyro<I2C>(i2c_master: &mut I2C) -> Result<(f32, f32, f32), I2C::Error>
where
    I2C: embedded_hal::i2c::I2c,
{
    i2c_master.write(SLAVE_ADDR, &[GYRO_XOUT_H])?;
    let mut buffer = [0; 6];
    i2c_master.read(SLAVE_ADDR, &mut buffer)?;
    Ok((
        conv((buffer[0] as i16) << 8 | buffer[1] as i16, 500.0),
        conv((buffer[2] as i16) << 8 | buffer[3] as i16, 500.0),
        conv((buffer[4] as i16) << 8 | buffer[5] as i16, 500.0),
    ))
}

fn conv(representation: i16, scale: f32) -> f32 {
    scale / i16::MAX as f32 * representation as f32
}

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

type SpiInterface<'a, 'b> = SPIInterfaceNoCS<
    SpiDeviceDriver<'a, &'a SpiDriver<'a>>,
    PinDriver<'b, esp_idf_hal::gpio::AnyOutputPin, esp_idf_hal::gpio::Output>,
>;

type DisplayType<'a, 'b> = mipidsi::Display<
    SpiInterface<'a, 'b>,
    mipidsi::models::ILI9342CRgb565,
    PinDriver<'b, esp_idf_hal::gpio::AnyOutputPin, esp_idf_hal::gpio::Output>,
>;

pub fn initialize_display<'a, 'b, 'c>(
    driver: &'c SpiDriver<'a>,
    config: SpiConfig,
    cs: esp_idf_hal::gpio::AnyOutputPin,
    dc: esp_idf_hal::gpio::AnyOutputPin,
) -> DisplayType<'a, 'b>
where
    'c: 'a,
{
    let lcd_spi_master =
        SpiDeviceDriver::new(driver, Some(cs), &config).expect("Failed to initialize SPI device");

    let dc = PinDriver::output(dc).expect("Failed to initialize DC pin");
    let spi_iface = SPIInterfaceNoCS::new(lcd_spi_master, dc);

    mipidsi::Builder::ili9342c_rgb565(spi_iface)
        .with_display_size(320, 240)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(
            &mut esp_idf_hal::delay::FreeRtos,
            None::<PinDriver<esp_idf_hal::gpio::AnyOutputPin, esp_idf_hal::gpio::Output>>,
        )
        .expect("Failed to initialize display")
}
