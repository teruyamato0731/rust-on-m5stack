extern crate nalgebra as na;
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
    uart::UartDriver,
};
use nalgebra::{matrix, vector};
use std::{f32::consts::PI, time::Instant};

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
    let mut ukf = init_ukf();

    let mut control = Control::default();

    let mut pre = Instant::now();
    let mut last_recv = Instant::now();
    loop {
        // let acc = imu.read_accel()?;
        let gyro = imu.read_gyro()?;

        // Receive a message
        if let Ok(frame) = can.receive() {
            c620.parse_packet(&frame);
        }

        // UARTから制御信号を受信
        if let Some(c) = read_control(&uart) {
            control = c;
            last_recv = Instant::now();
        };

        // 制御信号が一定時間受信されない場合、モータを停止
        if last_recv.elapsed() > core::time::Duration::from_millis(20) {
            control.u = 0;
        }

        ukf.predict(control.u as f32, fx);
        let x_obs = vector![c620.rx[0].rpm as f32, gyro.0 as f32];
        ukf.update(&x_obs, hx);

        // Send a message every 500 ms
        let wait = core::time::Duration::from_millis(500);
        if pre.elapsed() > wait {
            pre = Instant::now();

            c620.pwm[0] = control.u;
            c620.pwm[1] = -control.u;
            let frames = c620.to_msgs();
            for frame in frames.iter() {
                if let Err(e) = can.transmit(frame) {
                    log::error!("Error sending frame: {:?}", e);
                } else {
                    log::trace!("Sent frame {:?}", frame);
                }
            }

            // 状態変数を送信
            let s: State = ukf.state().into();
            let cobs = s.as_cobs();
            let _len = uart.write(&cobs)?;

            display.clear(Rgb565::BLACK).unwrap();

            // 数字を埋め込んで表示
            let text = format!(
                "control: {:6} {:6}\ngyro: ({:6.2},{:6.2},{:6.2})",
                control.u, c620.pwm[0], gyro.0, gyro.1, gyro.2
            );
            let style = MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::WHITE);
            // Draw with embedded_graphics
            Text::new(&text, Point::new(20, 30), style)
                .draw(&mut display)
                .unwrap();
        }
    }
}

// 状態遷移関数
const M1: f32 = 150e-3;
const R_W: f32 = 50e-3;
const M2: f32 = 2.3 - 2.0 * M1 + 2.0;
const L: f32 = 0.2474; // 重心までの距離
const J1: f32 = M1 * R_W * R_W;
const J2: f32 = 0.2;
const G: f32 = 9.81;
const KT: f32 = 0.15; // m2006
const D: f32 = (M1 + M2 + J1 / R_W * R_W) * (M2 * L * L + J2) - M2 * M2 * L * L;
const DT: f32 = 0.01;
fn fx(state: &na::Vector4<f32>, u: f32) -> na::Vector4<f32> {
    let mut x = *state;
    x[3] += ((M1 + M2 + J1 / R_W * R_W) / D * M2 * G * L * x[2] - M2 * L / D / R_W * KT * u) * DT;
    x[2] += x[3] * DT;
    x[1] += (-M2 * M2 * G * L * L / D * x[2] + (M2 * L * L + J2) / D / R_W * KT * u) * DT;
    x[0] += x[1] * DT;
    x
}

// 観測関数
fn hx(state: &na::Vector4<f32>) -> na::Vector2<f32> {
    vector![
        60.0 / (2.0 * PI * R_W) * state[1], // 駆動輪のオドメトリ [m/s] -> [rpm]
        state[3].to_radians(),              // 角速度 [deg/s] -> [rad/s]
    ]
}

fn read_control(uart: &UartDriver) -> Option<Control> {
    let mut buf = [0u8; 4];
    let timeout = TickType::new_millis(3);
    let len = uart.read(&mut buf, timeout.into()).unwrap();

    if len == 4 && buf[3] == 0 {
        Control::from_cobs(&buf)
    } else {
        None
    }
}

fn init_ukf() -> UnscentedKalmanFilter {
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
        2.0, 0.0;
        0.0, 200.0;
    ];
    UnscentedKalmanFilter::new(vector![0.0, 0.0, 0.0, 0.0], p, q, r)
}
