mod axp192;
mod c620;
mod mcp2515;

use c620::C620;
use core::cell::RefCell;
use core2::axp192::Axp192;
use core2::m5_core2::{
    i2c_master_init, imu_init, imu_read_accel, imu_read_gyro, initialize_display, m5sc2_init,
};
use embedded_can::nb::Can;
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal_bus::i2c;
use esp_idf_hal::{
    delay::{FreeRtos, TickType},
    prelude::*,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    uart::{UartConfig, UartDriver},
    units::Hertz,
};
use mcp2515::MCP2515;
use std::time::Instant;
use zerocopy::{AsBytes, FromBytes, FromZeroes};

// 状態変数 x, \dot{x}, \theta, \theta
#[derive(Debug, AsBytes, FromBytes, FromZeroes)]
#[repr(C)]
struct State {
    x: f32,
    dx: f32,
    theta: f32,
    dtheta: f32,
}

#[derive(Debug, AsBytes, FromBytes, FromZeroes)]
#[repr(C)]
struct Control {
    u: i16,
}

fn main() {
    esp_idf_svc::sys::link_patches();
    // esp_idf_svc::log::EspLogger::initialize_default();

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
    can.set_mode(mcp2515::Mode::Normal, &mut esp_idf_hal::delay::FreeRtos)?;

    // CANの設定完了
    log::debug!("CAN setup done!");

    let mut display = initialize_display(
        &driver,
        config,
        peripherals.pins.gpio5.into(),
        peripherals.pins.gpio15.into(),
    );

    // LCDの設定完了
    log::debug!("LCD setup done!");

    // UARTの初期化
    let tx = peripherals.pins.gpio1;
    let rx = peripherals.pins.gpio3;
    let uart_config = UartConfig::new().baudrate(Hertz(115_200));
    let uart = UartDriver::new(
        peripherals.uart0,
        tx,
        rx,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &uart_config,
    )?;

    // Make the display all green
    display
        .clear(Rgb565::GREEN)
        .expect("Failed to clear display");
    // Draw with embedded_graphics
    Text::with_alignment(
        "hinge",
        Point::new(160, 120),
        MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::BLACK),
        embedded_graphics::text::Alignment::Center,
    )
    .draw(&mut display)
    .expect("Failed to draw text");

    let mut c620 = C620::new();

    let mut pre = Instant::now();
    loop {
        let acc = imu_read_accel(&mut i2c::RefCellDevice::new(&i2c_ref_cell))?;
        let gyro = imu_read_gyro(&mut i2c::RefCellDevice::new(&i2c_ref_cell))?;

        // Receive a message
        match can.receive() {
            Ok(frame) => log::info!("Received frame {:?}", frame),
            Err(nb::Error::WouldBlock) => {
                log::trace!("No message to read!")
            }
            Err(e) => log::error!("Error receiving frame: {:?}", e),
        }

        // UARTからデータを読み取る
        let mut buf = [0u8; 4];
        let timeout = TickType::new_millis(1000);
        let len = uart.read(&mut buf, timeout.into())?;
        // println!("Received from UART: {:?}, len: {}", buf, len);

        // 4バイトの正常なデータを受信したら、制御入力として解釈
        let control = if len == 4 && buf[3] == 0 {
            let (buf, _): ([u8; 2], usize) = cobs_rs::unstuff(buf, 0);
            Control::read_from(&buf).unwrap()
        } else {
            Control { u: 0 }
        };

        c620.pwm[0] = (control.u / 2).clamp(-C620::PWM_MAX, C620::PWM_MAX);

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

            // 状態変数を送信
            let s = State {
                x: 1.2,
                dx: 3.4,
                theta: 5.6,
                dtheta: 7.8,
            };
            let buf = s.as_bytes();
            let cobs = cobs_rs::stuff::<16, 18>(buf.try_into().unwrap(), 0);
            let _len = uart.write(&cobs)?;

            let style = MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::WHITE);
            display.clear(Rgb565::BLACK).unwrap();
            // Draw with embedded_graphics
            Text::with_alignment(
                &format!("control: {}, {}", control.u, c620.pwm[0]),
                Point::new(160, 120),
                style,
                embedded_graphics::text::Alignment::Center,
            )
            .draw(&mut display)
            .unwrap();

            // 数字を埋め込んで表示
            // acc: %d\n gyro: %d
            let text = format!(
                " acc: ({:6.2},{:6.2},{:6.2})\ngyro: ({:6.2},{:6.2},{:6.2})",
                acc.0, acc.1, acc.2, gyro.0, gyro.1, gyro.2
            );

            Text::new(&text, Point::new(20, 30), style)
                .draw(&mut display)
                .unwrap();
        }
    }
}
