// use embedded_can::Frame;
// use embedded_can::StandardId;
// use esp_idf_hal::can;
// use esp_idf_hal::prelude::*;
// use esp_idf_hal::sys::EspError;

// fn main() {
//     esp_idf_svc::sys::link_patches();
//     esp_idf_svc::log::EspLogger::initialize_default();

//     run().unwrap();
// }

// fn run() -> Result<(), EspError> {
//     let peripherals = Peripherals::take().unwrap();
//     let pins = peripherals.pins;

//     // filter to accept only CAN ID 881
//     // let filter = can::config::Filter::Standard {
//     //     filter: 200,
//     //     mask: 0x7FF,
//     // };
//     // filter that accepts all CAN IDs
//     let filter = can::config::Filter::standard_allow_all();

//     let timing = can::config::Timing::B1M;
//     let config = can::config::Config::new().filter(filter).timing(timing);
//     let mut can = can::CanDriver::new(peripherals.can, pins.gpio5, pins.gpio4, &config).unwrap();

//     let tx_frame = Frame::new(StandardId::new(0x200).unwrap(), &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
//     can.transmit(&tx_frame, 1000).unwrap();

//     if let Ok(rx_frame) = can.receive(1000) {
//         print!("rx {:}:", rx_frame);
//     }

//     loop {
//         println!("Hello, world!");
//     }
// }
