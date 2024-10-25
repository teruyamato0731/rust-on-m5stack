use crate::mcp2515::CanFrame;
use embedded_can::{Frame, Id, StandardId};

#[derive(Default, Debug)]
pub struct C610Packet {
    pub angle: u16,
    pub rpm: i16,
    pub ampere: i16,
}

impl C610Packet {
    pub fn parse(&mut self, data: &[u8]) {
        self.angle = u16::from_be_bytes([data[0], data[1]]);
        self.rpm = i16::from_be_bytes([data[2], data[3]]);
        self.ampere = i16::from_be_bytes([data[4], data[5]]);
    }
}

#[derive(Default, Debug)]
pub struct C610 {
    pub pwm: [i16; 8],
    pub rx: [C610Packet; 8],
}

impl C610 {
    pub const PWM_MAX: i16 = 10000;

    pub fn new() -> Self {
        Self::default()
    }
    pub fn to_msgs(&self) -> [CanFrame; 2] {
        let mut data = [0; 16];
        for i in 0..8 {
            data[2 * i] = (self.pwm[i] >> 8) as u8;
            data[2 * i + 1] = self.pwm[i] as u8;
        }
        [
            CanFrame::new(StandardId::new(0x200).unwrap(), &data[0..8]).unwrap(),
            CanFrame::new(StandardId::new(0x199).unwrap(), &data[8..16]).unwrap(),
        ]
    }
    pub fn parse_packet(&mut self, frame: &CanFrame) {
        if let Id::Standard(id) = frame.id() {
            if frame.dlc() == 8
                && !frame.is_remote_frame()
                && 0x200 < id.as_raw()
                && id.as_raw() <= 0x208
            {
                let index = id.as_raw() - 0x201;
                self.rx[index as usize].parse(frame.data());
            }
        }
    }
}
