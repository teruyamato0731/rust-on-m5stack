use nalgebra::Vector4;
use zerocopy::{AsBytes, FromBytes, FromZeroes};

// 状態変数 x, \dot{x}, \theta, \theta
#[derive(Debug, AsBytes, FromBytes, FromZeroes, Default)]
#[repr(C)]
pub struct State {
    pub x: f32,
    pub dx: f32,
    pub theta: f32,
    pub dtheta: f32,
}

#[derive(Debug, AsBytes, FromBytes, FromZeroes, Default)]
#[repr(C)]
pub struct Control {
    pub u: i16,
}

#[derive(Debug, AsBytes, FromBytes, FromZeroes)]
#[repr(C)]
pub struct Sensor {
    pub encoder: f32, // 駆動輪のオドメトリ
    pub gyro: f32,    // ジャイロセンサの角速度
}

impl State {
    pub const SIZE: usize = std::mem::size_of::<Self>();
    pub const BUF_SIZE: usize = Self::SIZE + 2;

    // cobsバイト列に変換
    pub fn as_cobs(&self) -> [u8; Self::BUF_SIZE] {
        let buf: [u8; Self::SIZE] = self.as_bytes().try_into().unwrap();
        cobs_rs::stuff(buf, 0)
    }
    // cobsバイト列から復元
    pub fn from_cobs(buf: &[u8; Self::BUF_SIZE]) -> Option<Self> {
        let (cobs, _): ([u8; Self::SIZE], _) = cobs_rs::unstuff(*buf, 0);
        Self::read_from(&cobs)
    }
}

impl Control {
    pub const MAX: i16 = 16000;
    pub const SIZE: usize = std::mem::size_of::<Self>();
    pub const BUF_SIZE: usize = Self::SIZE + 2;

    // cobsバイト列に変換
    pub fn as_cobs(&self) -> [u8; Self::BUF_SIZE] {
        let buf: [u8; Self::SIZE] = self.as_bytes().try_into().unwrap();
        cobs_rs::stuff(buf, 0)
    }
    // cobsバイト列から復元
    pub fn from_cobs(buf: &[u8; Self::BUF_SIZE]) -> Option<Self> {
        let (cobs, _): ([u8; Self::SIZE], _) = cobs_rs::unstuff(*buf, 0);
        Self::read_from(&cobs)
    }
    pub fn from_current(current: f32) -> Self {
        const K: f32 = Control::MAX as f32 / 20.0;
        let u = (K * current) as i16;
        Control { u }
    }
    pub fn current(&self) -> f32 {
        const K: f32 = Control::MAX as f32 / 20.0;
        self.u as f32 / K
    }
}

impl Sensor {
    pub const SIZE: usize = std::mem::size_of::<Self>();
    pub const BUF_SIZE: usize = Self::SIZE + 2;

    // cobsバイト列に変換
    pub fn as_cobs(&self) -> [u8; Self::BUF_SIZE] {
        let buf: [u8; Self::SIZE] = self.as_bytes().try_into().unwrap();
        cobs_rs::stuff(buf, 0)
    }
    // cobsバイト列から復元
    pub fn from_cobs(buf: &[u8; Self::BUF_SIZE]) -> Option<Self> {
        let (cobs, _): ([u8; Self::SIZE], _) = cobs_rs::unstuff(*buf, 0);
        Self::read_from(&cobs)
    }
}

impl From<Vector4<f32>> for State {
    fn from(v: Vector4<f32>) -> Self {
        Self {
            x: v[0],
            dx: v[1],
            theta: v[2],
            dtheta: v[3],
        }
    }
}
