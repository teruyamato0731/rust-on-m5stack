use crate::mcp2515::CanFrame;
use embedded_can::{Frame, Id, StandardId};

#[derive(Default, Debug)]
pub struct C620Packet {
    pub angle: u16,
    pub rpm: i16,
    pub ampere: i16,
    pub temp: u8,
}

impl C620Packet {
    pub fn parse(data: &[u8]) -> Self {
        Self {
            angle: u16::from_be_bytes([data[0], data[1]]),
            rpm: i16::from_be_bytes([data[2], data[3]]),
            ampere: i16::from_be_bytes([data[4], data[5]]),
            temp: data[6],
        }
    }
}

#[derive(Default, Debug)]
pub struct C620 {
    pub pwm: [i16; 8],
    pub rx: [C620Packet; 8],
}

impl C620 {
    pub const PWM_MAX: i16 = 16000;

    pub fn new() -> Self {
        Self::default()
    }
    pub fn to_msgs(&self) -> [CanFrame; 2] {
        let mut data = [[0; 8]; 2];
        for i in 0..4 {
            data[0][2 * i] = (self.pwm[i] >> 8) as u8;
            data[0][2 * i + 1] = self.pwm[i] as u8;
            data[1][2 * i] = (self.pwm[4 + i] >> 8) as u8;
            data[1][2 * i + 1] = self.pwm[4 + i] as u8;
        }
        [
            CanFrame::new(StandardId::new(0x200).unwrap(), &data[0]).unwrap(),
            CanFrame::new(StandardId::new(0x199).unwrap(), &data[1]).unwrap(),
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
                self.rx[index as usize] = C620Packet::parse(frame.data());
            }
        }
    }
}
