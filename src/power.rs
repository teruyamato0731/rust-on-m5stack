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
        let reg_data_array = [
            (0x26, 0x6A),       // reg26h DCDC1 3350mV (ESP32 VDD)
            (0x30, 0b00000010), // reg30h VBUS-IPSOUT Pass-Through Management
            (0x31, 0b00000100), // reg31h VOFF Shutdown voltage setting (3.0V)
            (0x32, 0b01000010), // reg32h Enable bat detection
            (0x33, 0b11000000), // reg33h Charge control 1 (Charge 4.2V, 100mA)
            (0x35, 0xA2),       // reg35h Enable RTC BAT charge
            (0x36, 0x0C),       // reg36h 128ms power on, 4s power off
            (0x40, 0x00),       // reg40h IRQ 1, all disable
            (0x41, 0x00),       // reg41h IRQ 2, all disable
            (0x42, 0x03),       // reg42h IRQ 3, power key irq enable
            (0x43, 0x00),       // reg43h IRQ 4, all disable
            (0x44, 0x00),       // reg44h IRQ 5, all disable
            (0x82, 0xFF),       // reg82h ADC all on
            (0x83, 0x80),       // reg83h ADC temp on
            (0x84, 0x32),       // reg84h ADC 25Hz
            (0x90, 0x07),       // reg90h GPIO0(LDOio0) floating
            (0x91, 0xA0),       // reg91h GPIO0(LDOio0) 2.8V
            (0x92, 0x02),
            (0x98, 0x00), // PWM1 X
            (0x99, 0xFF), // PWM1 Y1
            (0x9A, 0xFF), // PWM1 Y1
        ];

        for &(reg, data) in &reg_data_array {
            self.axp.set_8(reg, data)?;
        }
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
