use esp_idf_hal::{
    delay::{FreeRtos, BLOCK},
    gpio::AnyIOPin,
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    prelude::*,
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

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    run().unwrap();
}

fn run() -> Result<(), EspError> {
    let peripherals = Peripherals::take()?;

    let mut i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21.into(),
        peripherals.pins.gpio22.into(),
        400.kHz().into(),
    )?;

    imu_init(&mut i2c_master)?;

    loop {
        let acc = imu_read_accel(&mut i2c_master);
        let gyro = imu_read_gyro(&mut i2c_master);
        print!("acc x:{:6.2}, y:{:6.2}, z:{:6.2} ", acc.0, acc.1, acc.2);
        print!("gyro x:{:4.0}, y:{:4.0}, z:{:4.0} ", gyro.0, gyro.1, gyro.2);
        println!();
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

fn imu_init(i2c_master: &mut I2cDriver) -> Result<(), EspError> {
    i2c_master.write(SLAVE_ADDR, &[WHOAMI], BLOCK)?;
    let mut tmp = [0];
    i2c_master.read(SLAVE_ADDR, &mut tmp, BLOCK)?;
    println!("WHOAMI: {:#X}", tmp[0]);
    assert_eq!(tmp[0], 0x19);

    // reset
    i2c_master.write(SLAVE_ADDR, &[PWR_MGMT_1, 0x80], BLOCK)?;
    // wait for reset
    FreeRtos::delay_ms(15);
    // activate gyro and accel
    i2c_master.write(SLAVE_ADDR, &[PWR_MGMT_1, 0x01], BLOCK)?;
    // set gyro range to +-500 deg/s and accel range to +-4g
    i2c_master.write(SLAVE_ADDR, &[CONFIG, 0x1A, 0x08, 0x08], BLOCK)?;

    Ok(())
}

fn imu_read_accel(i2c_master: &mut I2cDriver) -> (f32, f32, f32) {
    i2c_master
        .write(SLAVE_ADDR, &[ACCEL_XOUT_H], BLOCK)
        .unwrap();
    let mut buffer = [0; 6];
    i2c_master.read(SLAVE_ADDR, &mut buffer, BLOCK).unwrap();
    return (
        conv((buffer[0] as i16) << 8 | buffer[1] as i16, 4.0 * 9.8),
        conv((buffer[2] as i16) << 8 | buffer[3] as i16, 4.0 * 9.8),
        conv((buffer[4] as i16) << 8 | buffer[5] as i16, 4.0 * 9.8),
    );
}

fn imu_read_gyro(i2c_master: &mut I2cDriver) -> (f32, f32, f32) {
    i2c_master.write(SLAVE_ADDR, &[GYRO_XOUT_H], BLOCK).unwrap();
    let mut buffer = [0; 6];
    i2c_master.read(SLAVE_ADDR, &mut buffer, BLOCK).unwrap();
    return (
        conv((buffer[0] as i16) << 8 | buffer[1] as i16, 500.0),
        conv((buffer[2] as i16) << 8 | buffer[3] as i16, 500.0),
        conv((buffer[4] as i16) << 8 | buffer[5] as i16, 500.0),
    );
}

fn conv(representation: i16, scale: f32) -> f32 {
    scale / i16::MAX as f32 * representation as f32
}
