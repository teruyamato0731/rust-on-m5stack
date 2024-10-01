use std::time::Instant;

use embedded_can::{ErrorKind, Frame, Id, StandardId};
use embedded_hal::spi::{Operation, SpiDevice};

#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    /// ID of CAN frame.
    pub(crate) id: Id,
    /// Whether the frame is an RTR frame.
    pub(crate) rtr: bool,
    /// Length of data in CAN frame.
    pub(crate) dlc: u8,
    /// Data, maximum 8 bytes.
    pub(crate) data: [u8; 8],
}

impl Frame for CanFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        Some(Self {
            id: id.into(),
            rtr: false,
            dlc: data.len() as u8,
            data: {
                let mut data_bytes = [0; 8];
                data_bytes[..data.len()].copy_from_slice(data);
                data_bytes
            },
        })
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }

        Some(Self {
            id: id.into(),
            rtr: true,
            dlc: dlc as u8,
            data: [0; 8],
        })
    }

    fn is_extended(&self) -> bool {
        match self.id {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        self.rtr
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

#[derive(Clone, Copy)]
pub struct CanError {
    kind: ErrorKind,
}

impl CanError {
    pub fn new(kind: ErrorKind) -> Self {
        Self { kind }
    }
    pub fn other() -> Self {
        Self::new(ErrorKind::Other)
    }
}

impl core::fmt::Display for CanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CanError {{ kind: {} }}", self.kind)
    }
}

impl core::fmt::Debug for CanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CanError {{ kind: {:?} }}", self.kind)
    }
}

impl std::error::Error for CanError {}

impl embedded_can::Error for CanError {
    fn kind(&self) -> ErrorKind {
        self.kind
    }
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Mode {
    Normal = 0b000 << 5,
    Sleep = 0b001 << 5,
    Loopback = 0b010 << 5,
    ListenOnly = 0b011 << 5,
    Configuration = 0b100 << 5,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum TxBuf {
    Tx0 = 0,
    Tx1 = 1,
    Tx2 = 2,
}

#[allow(dead_code)]
impl TxBuf {
    fn ctrl(&self) -> u8 {
        0x30 + 0x10 * *self as u8
    }
    fn load(&self) -> u8 {
        0x40 + 0x02 * *self as u8
    }
    fn rts(&self) -> u8 {
        0x80 + (1 << *self as u8)
    }
    // STANDARD IDENTIFIER REGISTER
    fn sidh(&self) -> u8 {
        0x31 + 0x10 * *self as u8
    }
    // DATA LENGTH CODE
    fn dlc(&self) -> u8 {
        0x35 + 0x10 * *self as u8
    }
    // DATA BYTE 0
    fn data0(&self) -> u8 {
        0x36 + 0x10 * *self as u8
    }
}

fn frame_to_byte_stream(frame: &CanFrame) -> [u8; 13] {
    let id = match frame.id {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(_) => unimplemented!("Extended ID is not supported"),
    };
    let id_raw = id << 5;
    let id_padding = [0; 2]; // ext id
    let dlc = (frame.rtr as u8) << 6 | frame.dlc;
    let iter = id_raw
        .to_be_bytes()
        .into_iter()
        .chain(id_padding)
        .chain([dlc])
        .chain(frame.data);
    let mut data = [0; 13];
    for (i, byte) in iter.enumerate() {
        data[i] = byte;
    }
    data
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum RxBuf {
    Rx0 = 0,
    Rx1 = 1,
}

#[allow(dead_code)]
impl RxBuf {
    fn ctrl(&self) -> u8 {
        0x60 + 0x10 * *self as u8
    }
    // STANDARD IDENTIFIER REGISTER
    fn sidh(&self) -> u8 {
        0x61 + 0x10 * *self as u8
    }
    // DATA LENGTH CODE
    fn dlc(&self) -> u8 {
        0x65 + 0x10 * *self as u8
    }
    // DATA BYTE 0
    fn data0(&self) -> u8 {
        0x66 + 0x10 * *self as u8
    }

    fn read_rx_buf(&self) -> u8 {
        0x90 + 0x04 * *self as u8
    }
}

pub struct MCP2515<SPI> {
    spi: SPI,
}

#[allow(dead_code)]
impl<SPI> MCP2515<SPI>
where
    SPI: SpiDevice,
{
    // Instruction
    const MCP_WRITE: u8 = 0x02;
    const MCP_READ: u8 = 0x03;
    const MCP_MODIFY: u8 = 0x05;
    const MCP_CANSTAT: u8 = 0x0E;
    const MCP_CANCTRL: u8 = 0x0F;
    // Address
    const MCP_CNF1: u8 = 0x2A;
    const MCP_CNF2: u8 = 0x29;
    const MCP_CNF3: u8 = 0x28;
    const CANINTE: u8 = 0x2B;
    const CANINTF: u8 = 0x2C;
    const TXB0CTRL: u8 = 0x30;
    const TXB1CTRL: u8 = 0x40;
    const TXB2CTRL: u8 = 0x50;
    const RXB0CTRL: u8 = 0x60;
    const RXB1CTRL: u8 = 0x70;
    // Mask
    const TXREQ_MASK: u8 = 0x08;
    const ID_EXTENDED_MASK: u8 = 0x08;
    const MODE_MASK: u8 = 0xE0;
    const RXB_RX_MASK: u8 = 0x60;
    const RXB_BUKT_MASK: u8 = 0x04;
    const RXB_RX_ANY: u8 = 0x60;

    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn reset(&mut self, delay: &mut impl embedded_hal::delay::DelayNs) -> Result<(), CanError> {
        const RESET: u8 = 0xC0;
        self.spi
            .transaction(&mut [Operation::Write(&[RESET])])
            .map_err(|_| CanError::other())?;
        delay.delay_ms(5);
        Ok(())
    }

    pub fn init(&mut self, delay: &mut impl embedded_hal::delay::DelayNs) -> Result<(), CanError> {
        self.reset(delay)?;
        self.set_mode(Mode::Configuration, delay)?;

        // 1MHz に設定
        self.set_baudrate(1_000_000)?;
        log::trace!("Baudrate set to 1MHz");

        // // CLKOUT ピンを無効化
        // self.set_clken(false)?;

        // Clear all filters
        self.write_registers(0x20, &[0; 4])?;
        self.write_registers(0x24, &[0; 4])?;
        self.write_registers(0x00, &[0; 4])?;
        self.write_registers(0x04, &[0; 4])?;
        self.write_registers(0x08, &[0; 4])?;
        self.write_registers(0x10, &[0; 4])?;
        self.write_registers(0x14, &[0; 4])?;
        self.write_registers(0x18, &[0; 4])?;

        // Clear Tx registers (TXB{O,1,2}CTRL += 14)
        self.write_registers(Self::TXB0CTRL, &[0; 14])?;
        self.write_registers(Self::TXB1CTRL, &[0; 14])?;
        self.write_registers(Self::TXB2CTRL, &[0; 14])?;

        // Clear Rx registers
        self.write_registers(Self::RXB0CTRL, &[0])?;
        self.write_registers(Self::RXB1CTRL, &[0])?;

        // can interrupt enable
        self.write_registers(Self::CANINTE, &[0x03])?;

        //Sets BF pins as GPO
        self.write_registers(0x0C, &[0x30 | 0x0C])?;
        //Sets RTS pins as GPI
        self.write_registers(0x0D, &[0x00])?;

        // すべて受取る
        self.modify_register(
            Self::RXB0CTRL,
            Self::RXB_RX_MASK | Self::RXB_BUKT_MASK,
            Self::RXB_RX_ANY | Self::RXB_BUKT_MASK,
        )?;
        self.modify_register(Self::RXB1CTRL, Self::RXB_RX_MASK, Self::RXB_RX_ANY)?;

        // Loopback mode を要求
        self.set_mode(Mode::Loopback, delay)?;
        Ok(())
    }

    pub fn set_baudrate(&mut self, baudrate: u32) -> Result<(), CanError> {
        if baudrate != 1_000_000 {
            unimplemented!("Only 1MHz is supported")
        }
        // 1MHz に設定
        // self.write_registers(Self::MCP_CNF3, &[0x00, 0xC0, 0x80])?;
        self.write_registers(Self::MCP_CNF1, &[0x00])?;
        self.write_registers(Self::MCP_CNF2, &[0xC0])?;
        self.write_registers(Self::MCP_CNF3, &[0x80])?;
        Ok(())
    }

    pub fn set_mode(
        &mut self,
        mode: Mode,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), CanError> {
        self.request_mode(mode)?;
        let start = Instant::now();
        loop {
            delay.delay_us(100);
            let configured = self.verify_mode(mode)?;
            if configured {
                log::trace!("Mode verified to Normal");
                return Ok(());
            } else if start.elapsed().as_millis() > 200 {
                panic!("Mode verification failed");
                // return Err(CanError::other());
            }
        }
    }

    fn request_mode(&mut self, mode: Mode) -> Result<(), CanError> {
        // mode が Sleep だとエラー
        if let Mode::Sleep = mode {
            unimplemented!("Sleep mode is not supported")
        }
        let mut buf = [0; 1];
        self.read_registers(Self::MCP_CANSTAT, &mut buf)?;
        if buf[0] & Self::MODE_MASK == Mode::Sleep as u8 {
            unimplemented!("Sleep mode is not supported")
        }

        // Clear wake flag
        self.modify_register(Self::CANINTF, 0x40, 0)?;

        self.modify_register(Self::MCP_CANCTRL, Self::MODE_MASK, mode as u8)?;
        Ok(())
    }

    fn verify_mode(&mut self, mode: Mode) -> Result<bool, CanError> {
        let mut buf = [0; 1];
        self.read_registers(Self::MCP_CANSTAT, &mut buf)?;
        Ok(buf[0] == (mode as u8 & Self::MODE_MASK))
    }

    fn set_clken(&mut self, clken: bool) -> Result<(), CanError> {
        self.modify_register(Self::MCP_CANCTRL, 0x04, (clken as u8) << 2)
    }

    pub fn write_registers(&mut self, address: u8, values: &[u8]) -> Result<(), CanError> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[Self::MCP_WRITE, address]),
                Operation::Write(values),
            ])
            .map_err(|_| CanError::other())
    }

    pub fn read_registers(&mut self, address: u8, values: &mut [u8]) -> Result<(), CanError> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[Self::MCP_READ, address]),
                Operation::Read(values),
            ])
            .map_err(|_| CanError::other())
    }

    pub fn modify_register(&mut self, address: u8, mask: u8, data: u8) -> Result<(), CanError> {
        self.spi
            .transaction(&mut [Operation::Write(&[Self::MCP_MODIFY, address, mask, data])])
            .map_err(|_| CanError::other())
    }

    fn find_tx_buf(&mut self) -> nb::Result<TxBuf, CanError> {
        let start = Instant::now();
        loop {
            for tx_buf in [TxBuf::Tx0, TxBuf::Tx1, TxBuf::Tx2] {
                let mut buf = [0; 1];
                self.read_registers(tx_buf.ctrl(), &mut buf)?;
                if buf[0] & Self::TXREQ_MASK == 0 {
                    return Ok(tx_buf);
                }
            }
            if start.elapsed().as_micros() > 2500 {
                return Err(nb::Error::WouldBlock);
            }
        }
    }

    fn find_rx_buf(&mut self, status: u8) -> Option<RxBuf> {
        if status & 0x01 != 0 {
            Some(RxBuf::Rx0)
        } else if status & 0x02 != 0 {
            Some(RxBuf::Rx1)
        } else {
            None
        }
    }
}

impl<SPI> embedded_can::nb::Can for MCP2515<SPI>
where
    SPI: SpiDevice,
{
    type Error = CanError;
    type Frame = CanFrame;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        // TXBnCTRL を読んで TXREQ が立っていないものを探す
        let tx_buf = self.find_tx_buf()?;
        // Load TX Buffer Instruction
        let load_tx_load = tx_buf.load();
        let tx_steam = frame_to_byte_stream(frame);
        self.spi
            .transaction(&mut [
                Operation::Write(&[load_tx_load]),
                Operation::Write(&tx_steam),
            ])
            .map_err(|_| CanError::other())?;
        // RTS (Request to Send) Command
        // let rts = tx_buf.rts();
        // self.spi
        //     .transaction(&mut [Operation::Write(&[rts])])
        //     .map_err(|_| CanError::other())?;

        // Set `txreq` bit in ctrl register.
        self.modify_register(tx_buf.ctrl(), Self::TXREQ_MASK, Self::TXREQ_MASK)?;

        let start = Instant::now();
        loop {
            let mut buf = [0; 1];
            self.read_registers(tx_buf.ctrl(), &mut buf)?;
            if buf[0] & Self::TXREQ_MASK == 0 {
                return Ok(None);
            }
            if start.elapsed().as_micros() > 2500 {
                // Check for any errors.
                const ABTF: u8 = 0x40;
                const MLOA: u8 = 0x20;
                const TXERR: u8 = 0x10;
                if buf[0] & ABTF != 0 || buf[0] & MLOA != 0 || buf[0] & TXERR != 0 {
                    log::warn!("Error in transmission: {:#04x}", buf[0]);
                    return Err(CanError::other().into());
                } else {
                    return Err(nb::Error::WouldBlock);
                }
            }
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        // READ STATUS INSTRUCTION
        const READ_STATUS: u8 = 0xA0;
        let mut buf = [0; 1];
        self.spi
            .transaction(&mut [Operation::Write(&[READ_STATUS]), Operation::Read(&mut buf)])
            .map_err(|_| CanError::other())?;
        let rx_buf = self.find_rx_buf(buf[0]).ok_or(nb::Error::WouldBlock)?;
        // READ RX BUFFER INSTRUCTION
        let read_rx_buf = rx_buf.read_rx_buf();
        let mut id_byte = [0; 2];
        let mut id_padding = [0; 2];
        let mut dlc = [0; 1];
        let mut buf = [0; 8];
        self.spi
            .transaction(&mut [
                Operation::Write(&[read_rx_buf]),
                Operation::Read(&mut id_byte),
                Operation::Read(&mut id_padding), // ext id
                Operation::Read(&mut dlc),
                Operation::Read(&mut buf),
            ])
            .map_err(|_| CanError::other())?;
        let is_extended = id_byte[1] & Self::ID_EXTENDED_MASK != 0;
        if is_extended {
            unimplemented!("Extended ID is not supported")
        }
        let dlc = (dlc[0] & 0xF) as usize;
        let rtr = if is_extended {
            dlc & 0x40 != 0
        } else {
            id_byte[1] & 0x10 != 0
        };
        if rtr {
            unimplemented!("RTR is not supported");
        }
        let id = u16::from_be_bytes(id_byte);
        let id = StandardId::new(id >> 5).ok_or(nb::Error::Other(CanError::other()))?;
        let frame = CanFrame::new(id, &buf[0..dlc]);
        match frame {
            Some(frame) => Ok(frame),
            None => Err(CanError::other().into()),
        }
    }
}
