use core::cell::RefCell;
use core2::{
    c620::C620,
    m5_core2::{i2c_master_init, initialize_can, initialize_display, initialize_uart},
    mpu6886::Mpu6886,
    packet::{Control, State},
    power::Power,
    ukf::UnscentedKalmanFilter,
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
    spi::{SpiConfig, SpiDriver, SpiDriverConfig},
};
use nalgebra::{matrix, vector};
use std::time::Instant;
use zerocopy::{AsBytes, FromBytes};

fn main() {
    esp_idf_svc::sys::link_patches();
    // esp_idf_svc::log::EspLogger::initialize_default();

    run().unwrap();
}

fn run() -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;

    // init Peripherals begin
    let i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        400.kHz().into(),
    )?;
    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio18,
        peripherals.pins.gpio23,
        Some(peripherals.pins.gpio38),
        &SpiDriverConfig::new(),
    )?;
    let uart = initialize_uart(
        peripherals.uart0,
        peripherals.pins.gpio1,
        peripherals.pins.gpio3,
    )?;
    // init Peripherals end

    // init i2c device begin
    let i2c_ref_cell = RefCell::new(i2c_master);
    let _ = Power::new(i2c::RefCellDevice::new(&i2c_ref_cell)).init(&mut FreeRtos)?;
    let mut imu = Mpu6886::new(i2c::RefCellDevice::new(&i2c_ref_cell)).init()?;
    // init i2c device end

    // init spi device begin
    let config = SpiConfig::new().baudrate(10.MHz().into());
    let mut can = initialize_can(&spi_driver, peripherals.pins.gpio27, &config);
    let mut display = initialize_display(
        &spi_driver,
        &config,
        peripherals.pins.gpio5.into(),
        peripherals.pins.gpio15.into(),
    );
    // init spi device end

    let mut c620 = C620::new();
    let p = matrix![
        10.0, 0.0, 0.0, 0.0;
        0.0, 10.0, 0.0, 0.0;
        0.0, 0.0, 10.0, 0.0;
        0.0, 0.0, 0.0, 10.0;
    ];
    let q = matrix![
        0.0, 0.0, 0.0, 0.0;
        0.0, 1.0, 0.0, 0.0;
        0.0, 0.0, 0.25, 0.5;
        0.0, 0.0, 0.5, 1.0;
    ];
    let r = matrix![
        0.5, 0.0;
        0.0, 0.5;
    ];
    let mut _ukf = UnscentedKalmanFilter::new(vector![0.0, 0.0, 0.0, 0.0], p, q, r);

    let mut pre = Instant::now();
    loop {
        let acc = imu.read_accel()?;
        let gyro = imu.read_gyro()?;

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
        let timeout = TickType::new_millis(20);
        let len = uart.read(&mut buf, timeout.into())?;

        // 4バイトの正常なデータを受信したら、制御入力として解釈
        let control = if len == 4 && buf[3] == 0 {
            let (buf, _): ([u8; 2], usize) = cobs_rs::unstuff(buf, 0);
            Control::read_from(&buf).unwrap()
        } else {
            Control { u: 0 }
        };

        c620.pwm[0] = (control.u / 2).clamp(-C620::PWM_MAX, C620::PWM_MAX);

        // ukf.predict(control.u, fx);
        // ukf.update(&x_obs, hx);

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
