use anyhow::Context as _;
use core2::axp192::Axp192;
use core2::mcp2515::{CanFrame, MCP2515};
use embedded_can::nb::Can;
use embedded_can::{Frame, StandardId};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::{
    gpio::AnyIOPin,
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    units::Hertz,
};
use std::time::Instant;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    run().unwrap();
}

fn run() -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;

    // setup
    println!("setup start!");

    let i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21.into(),
        peripherals.pins.gpio22.into(),
        400.kHz().into(),
    )?;

    // let i2c_ref_cell = RefCell::new(i2c_master);
    // let mut axp = Axp192::new(i2c::RefCellDevice::new(&i2c_ref_cell));

    let mut axp = Axp192::new(i2c_master);
    m5sc2_init(&mut axp, &mut FreeRtos).unwrap();

    // 電源の設定完了
    println!("Power setup done!");

    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio18;
    let serial_out = peripherals.pins.gpio23; // SDO
    let serial_in = peripherals.pins.gpio38; // SDI
    let cs_1 = peripherals.pins.gpio27;

    let driver = SpiDriver::new(
        spi,
        sclk,
        serial_out,
        Some(serial_in),
        &SpiDriverConfig::new(),
    )?;

    // SPIの設定完了
    println!("SPI setup done!");

    let config = SpiConfig::new().baudrate(20.MHz().into());
    let can_spi_master = SpiDeviceDriver::new(&driver, Some(cs_1), &config)?;

    let mut can = MCP2515::new(can_spi_master);
    can.init(&mut esp_idf_hal::delay::FreeRtos)?;

    // CANの設定完了
    println!("CAN setup done!");

    let mut pre = Instant::now();
    loop {
        // Receive a message
        match can.receive() {
            Ok(frame) => println!("Received frame {:?}", frame),
            Err(nb::Error::WouldBlock) => println!("No message to read!"),
            Err(e) => println!("Error receiving frame: {:?}", e),
        }

        // Send a message every 10 ms
        let wait = core::time::Duration::from_millis(100);
        if pre.elapsed() > wait {
            pre = Instant::now();

            // Send a message
            let frame = CanFrame::new(
                StandardId::new(0x200).context("Failed to create standard ID")?,
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            )
            .context("Failed to create frame")?;
            if let Err(e) = can.transmit(&frame) {
                println!("Error sending frame: {:?}", e);
            } else {
                println!("Sent frame {:?}", frame);
            }
        }

        FreeRtos::delay_ms(10);
    }
}

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> anyhow::Result<I2cDriver<'d>> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

fn m5sc2_init<I2C>(
    axp: &mut core2::axp192::Axp192<I2C>,
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

    axp.set_gpio1_mode(core2::axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
    axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                  // expect

    axp.set_gpio2_mode(core2::axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
    axp.set_gpio2_output(true)?;

    axp.set_key_mode(
        // Configure how the power button press will work
        core2::axp192::ShutdownDuration::Sd4s,
        core2::axp192::PowerOkDelay::Delay64ms,
        true,
        core2::axp192::LongPress::Lp1000ms,
        core2::axp192::BootTime::Boot512ms,
    )?;

    axp.set_gpio4_mode(core2::axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

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
