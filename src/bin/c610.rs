extern crate nalgebra as na;
use core::cell::RefCell;
use core2::{
    c610::C610,
    m5_core2::{i2c_master_init, initialize_can},
    power::Power,
};
use embedded_can::nb::Can;
use embedded_can::Frame;
use embedded_hal_bus::i2c;
use esp_idf_hal::{
    delay::FreeRtos,
    prelude::*,
    spi::{SpiConfig, SpiDriver, SpiDriverConfig},
};
use std::time::Instant;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

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
    // init Peripherals end

    // init i2c device begin
    let i2c_ref_cell = RefCell::new(i2c_master);
    let _ = Power::new(i2c::RefCellDevice::new(&i2c_ref_cell)).init(&mut FreeRtos)?;
    // init i2c device end

    // init spi device begin
    let config = SpiConfig::new().baudrate(10.MHz().into());
    let mut can = initialize_can(&spi_driver, peripherals.pins.gpio27, &config);
    // init spi device end

    let mut c610 = C610::new();

    let mut pre = Instant::now();
    loop {
        // Receive a message
        if let Ok(frame) = can.receive() {
            let rpm = i16::from_be_bytes([frame.data()[2], frame.data()[3]]);
            let raw = u16::from_be_bytes([frame.data()[4], frame.data()[5]]);

            println!(
                "rpm: {:8}, raw: {:08x}, Received frame {:?}",
                rpm, raw, frame
            );
            c610.parse_packet(&frame);
        }

        let wait = core::time::Duration::from_millis(30);
        if pre.elapsed() > wait {
            pre = Instant::now();

            c610.pwm[0] = 10000;
            c610.pwm[1] = 10000;
            let frames = c610.to_msgs();
            for frame in frames.iter() {
                if let Err(e) = can.transmit(frame) {
                    log::error!("Error sending frame: {:?}", e);
                } else {
                    log::trace!("Sent frame {:?}", frame);
                }
            }
        }
    }
}
