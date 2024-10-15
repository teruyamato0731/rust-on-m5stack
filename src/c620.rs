use crate::mcp2515::CanFrame;
use embedded_can::{Frame, StandardId};

pub struct C620 {
    pub pwm: [i16; 8],
}

impl C620 {
    pub const PWM_MAX: i16 = 16000;

    pub fn new() -> Self {
        Self { pwm: [0; 8] }
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
}
