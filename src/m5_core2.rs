use crate::axp192;
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::AnyIOPin,
    i2c::{I2c, I2cConfig, I2cDriver},
    peripheral::Peripheral,
    prelude::*,
    sys::EspError,
    units::Hertz,
};

struct Power<'a> {
    axp: Option<axp192::Axp192<I2cDriver<'a>>>,
}
struct Lcd {}
struct Imu {}

pub struct M5Core2<'a> {
    pub power: Power<'a>,
    pub display: Lcd,
    pub imu: Imu,
}

impl<'a> M5Core2<'a> {
    pub fn begin() -> Self {
        Self {
            power: Power { axp: None },
            display: Lcd {},
            imu: Imu {},
        }
    }
}

impl<'a> Power<'a> {
    pub fn begin(&mut self) -> Result<(), EspError> {
        let peripherals = Peripherals::take()?;
        let i2c_master = i2c_master_init(
            peripherals.i2c0,
            peripherals.pins.gpio21.into(),
            peripherals.pins.gpio22.into(),
            400.kHz().into(),
        )?;

        let mut axp = axp192::Axp192::new(i2c_master);
        m5sc2_init(&mut axp, &mut FreeRtos).expect("Failed to initialize AXP192");
        self.axp = Some(axp);

        // 電源の設定完了
        log::debug!("Power setup done!");
        Ok(())
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

fn m5sc2_init<I2C>(
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
