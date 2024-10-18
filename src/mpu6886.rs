use esp_idf_hal::delay::FreeRtos;

pub struct Mpu6886<I2C> {
    i2c: I2C,
}

impl<I2C> Mpu6886<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    const SLAVE_ADDR: u8 = 0x68;
    const CONFIG: u8 = 0x1A;
    // const GYRO_CONFIG: u8 = 0x1B;
    // const ACCEL_CONFIG: u8 = 0x1C;
    const WHOAMI: u8 = 0x75;
    const PWR_MGMT_1: u8 = 0x6B;
    const ACCEL_XOUT_H: u8 = 0x3B;
    const GYRO_XOUT_H: u8 = 0x43;

    pub fn new(i2c: I2C) -> Self {
        Mpu6886 { i2c }
    }

    pub fn init(mut self) -> Result<Self, I2C::Error> {
        self.i2c.write(Self::SLAVE_ADDR, &[Self::WHOAMI])?;
        let mut tmp = [0];
        self.i2c.read(Self::SLAVE_ADDR, &mut tmp)?;
        log::debug!("WHOAMI: {:#X}", tmp[0]);
        assert_eq!(tmp[0], 0x19);

        // reset
        self.i2c
            .write(Self::SLAVE_ADDR, &[Self::PWR_MGMT_1, 0x80])?;
        // wait for reset
        FreeRtos::delay_ms(15);
        // activate gyro and accel
        self.i2c
            .write(Self::SLAVE_ADDR, &[Self::PWR_MGMT_1, 0x01])?;
        // set gyro range to +-500 deg/s and accel range to +-4g
        self.i2c
            .write(Self::SLAVE_ADDR, &[Self::CONFIG, 0x1A, 0x08, 0x08])?;

        Ok(self)
    }

    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        self.i2c.write(Self::SLAVE_ADDR, &[Self::ACCEL_XOUT_H])?;
        let mut buffer = [0; 6];
        self.i2c.read(Self::SLAVE_ADDR, &mut buffer)?;
        Ok((
            Self::conv((buffer[0] as i16) << 8 | buffer[1] as i16, 4.0 * 9.8),
            Self::conv((buffer[2] as i16) << 8 | buffer[3] as i16, 4.0 * 9.8),
            Self::conv((buffer[4] as i16) << 8 | buffer[5] as i16, 4.0 * 9.8),
        ))
    }

    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        self.i2c.write(Self::SLAVE_ADDR, &[Self::GYRO_XOUT_H])?;
        let mut buffer = [0; 6];
        self.i2c.read(Self::SLAVE_ADDR, &mut buffer)?;
        Ok((
            Self::conv((buffer[0] as i16) << 8 | buffer[1] as i16, 500.0),
            Self::conv((buffer[2] as i16) << 8 | buffer[3] as i16, 500.0),
            Self::conv((buffer[4] as i16) << 8 | buffer[5] as i16, 500.0),
        ))
    }

    fn conv(representation: i16, scale: f32) -> f32 {
        scale / i16::MAX as f32 * representation as f32
    }
}
