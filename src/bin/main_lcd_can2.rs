use anyhow::anyhow;
use core::cell::RefCell;
use core2::axp192::Axp192;
use core2::mcp2515::{CanFrame, MCP2515};
use display_interface_spi::SPIInterfaceNoCS;
use embedded_can::nb::Can;
use embedded_can::{Frame, StandardId};
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal_bus::i2c;
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{AnyIOPin, PinDriver},
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    prelude::*,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    sys::EspError,
    units::Hertz,
};
use std::time::Instant;

const SLAVE_ADDR: u8 = 0x68;

const CONFIG: u8 = 0x1A;
// const GYRO_CONFIG: u8 = 0x1B;
// const ACCEL_CONFIG: u8 = 0x1C;
const WHOAMI: u8 = 0x75;
const PWR_MGMT_1: u8 = 0x6B;
const ACCEL_XOUT_H: u8 = 0x3B;
const GYRO_XOUT_H: u8 = 0x43;

struct C620 {
    pwm: [i16; 8],
}

impl C620 {
    const PWM_MAX: i16 = 16000;

    fn new() -> Self {
        Self { pwm: [0; 8] }
    }
    fn to_msgs(&self) -> [CanFrame; 2] {
        let mut data = [[0; 8]; 2];
        for i in 0..4 {
            data[0][2 * i] = (self.pwm[i] >> 8) as u8;
            data[0][2 * i + 1] = self.pwm[i] as u8;
            data[1][2 * i] = (self.pwm[4 + i] >> 8) as u8;
            data[1][2 * i + 1] = self.pwm[4 + i] as u8;
        }
        [
            CanFrame::new(StandardId::new(0x200).unwrap(), &data[0]).unwrap(),
            CanFrame::new(StandardId::new(0x199).unwrap(), &data[1]).unwrap(),
        ]
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    run().unwrap();
}

fn run() -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;

    // setup
    log::debug!("setup start!");

    let i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21.into(),
        peripherals.pins.gpio22.into(),
        400.kHz().into(),
    )?;

    let i2c_ref_cell = RefCell::new(i2c_master);

    // let mut axp = Axp192::new(i2c_master);
    let mut axp = Axp192::new(i2c::RefCellDevice::new(&i2c_ref_cell));
    m5sc2_init(&mut axp, &mut FreeRtos)?;

    // 電源の設定完了
    log::debug!("Power setup done!");

    imu_init(&mut i2c::RefCellDevice::new(&i2c_ref_cell))?;

    // IMUの設定完了
    log::debug!("IMU setup done!");

    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio18;
    let serial_in = peripherals.pins.gpio38; // SDI
    let serial_out = peripherals.pins.gpio23; // SDO
    let cs_lcd = peripherals.pins.gpio5;
    let cs_can = peripherals.pins.gpio27;

    let driver = SpiDriver::new(
        spi,
        sclk,
        serial_out,
        Some(serial_in),
        &SpiDriverConfig::new(),
    )?;

    let config = SpiConfig::new().baudrate(10.MHz().into());
    let can_spi_master = SpiDeviceDriver::new(&driver, Some(cs_can), &config)?;

    let mut can = MCP2515::new(can_spi_master);
    can.init(&mut esp_idf_hal::delay::FreeRtos)?;
    can.set_mode(
        core2::mcp2515::Mode::Normal,
        &mut esp_idf_hal::delay::FreeRtos,
    )?;

    // CANの設定完了
    log::debug!("CAN setup done!");

    let lcd_spi_master = SpiDeviceDriver::new(&driver, Some(cs_lcd), &config)?;
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
        .map_err(|e| anyhow!("Error initializing display: {:?}", e))?;

    // LCDの設定完了
    log::debug!("LCD setup done!");

    // Make the display all green
    display
        .clear(Rgb565::GREEN)
        .map_err(|e| anyhow!("Error clearing display: {:?}", e))?;
    // Draw with embedded_graphics
    Text::with_alignment(
        "hinge",
        Point::new(160, 120),
        MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::BLACK),
        embedded_graphics::text::Alignment::Center,
    )
    .draw(&mut display)
    .map_err(|e| anyhow!("Error drawing text: {:?}", e))?;

    let mut c620 = C620::new();

    for e in c620.pwm.iter_mut() {
        *e = -C620::PWM_MAX / 4;
    }
    println!("{:?}", c620.pwm);

    let mut pre = Instant::now();
    loop {
        let acc = imu_read_accel(&mut i2c::RefCellDevice::new(&i2c_ref_cell))?;
        let gyro = imu_read_gyro(&mut i2c::RefCellDevice::new(&i2c_ref_cell))?;
        print!("acc x:{:6.2}, y:{:6.2}, z:{:6.2} ", acc.0, acc.1, acc.2);
        print!("gyro x:{:4.0}, y:{:4.0}, z:{:4.0} ", gyro.0, gyro.1, gyro.2);
        println!();

        // Receive a message
        match can.receive() {
            Ok(frame) => log::info!("Received frame {:?}", frame),
            Err(nb::Error::WouldBlock) => {
                log::trace!("No message to read!")
            }
            Err(e) => log::error!("Error receiving frame: {:?}", e),
        }

        // Send a message every 500 ms
        let wait = core::time::Duration::from_millis(500);
        if pre.elapsed() > wait {
            pre = Instant::now();

            let frames = c620.to_msgs();
            for frame in frames.iter() {
                if let Err(e) = can.transmit(frame) {
                    log::error!("Error sending frame: {:?}", e);
                } else {
                    log::info!("Sent frame {:?}", frame);
                }
            }
        }

        FreeRtos::delay_ms(30);
    }
}

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> Result<I2cDriver<'d>, EspError> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

fn imu_init<I2C>(i2c_master: &mut I2C) -> Result<(), I2C::Error>
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

fn imu_read_accel<I2C>(i2c_master: &mut I2C) -> Result<(f32, f32, f32), I2C::Error>
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

fn imu_read_gyro<I2C>(i2c_master: &mut I2C) -> Result<(f32, f32, f32), I2C::Error>
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
