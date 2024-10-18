use crate::axp192::{self, Axp192};
use embedded_hal::i2c::I2c;

pub struct Power<I2C> {
    axp: Axp192<I2C>,
}

impl<I2C> Power<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            axp: Axp192::new(i2c),
        }
    }

    pub fn init(
        mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<Self, I2C::Error> {
        // Default setup for M5Stack Core 2
        self.axp.set_dcdc1_voltage(3350)?; // Voltage to provide to the microcontroller (this one!)

        self.axp.set_ldo2_voltage(3300)?; // Peripherals (LCD, ...)
        self.axp.set_ldo2_on(true)?;

        self.axp.set_ldo3_voltage(2000)?; // Vibration motor
        self.axp.set_ldo3_on(false)?;

        self.axp.set_dcdc3_voltage(2800)?; // LCD backlight
        self.axp.set_dcdc3_on(true)?;

        self.axp
            .set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
        self.axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                           // expect

        self.axp
            .set_gpio2_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
        self.axp.set_gpio2_output(true)?;

        self.axp.set_key_mode(
            // Configure how the power button press will work
            axp192::ShutdownDuration::Sd4s,
            axp192::PowerOkDelay::Delay64ms,
            true,
            axp192::LongPress::Lp1000ms,
            axp192::BootTime::Boot512ms,
        )?;

        self.axp
            .set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

        self.axp.set_battery_voltage_adc_enable(true)?;
        self.axp.set_battery_current_adc_enable(true)?;
        self.axp.set_acin_current_adc_enable(true)?;
        self.axp.set_acin_voltage_adc_enable(true)?;

        // Actually reset the LCD
        self.axp.set_gpio4_output(false)?;
        self.axp.set_ldo3_on(true)?; // Buzz the vibration motor while intializing ¯\_(ツ)_/¯
        delay.delay_ms(100u32);
        self.axp.set_gpio4_output(true)?;
        self.axp.set_ldo3_on(false)?;
        delay.delay_ms(100u32);
        Ok(self)
    }
}
