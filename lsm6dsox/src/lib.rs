#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

pub mod registers;
pub mod configs;
pub mod types;

use registers::*;
pub use crate::types::*;

use core::fmt::{Debug, Formatter};

use embedded_hal::delay::*;

use embedded_hal::i2c::{I2c, SevenBitAddress};
use heapless::spsc::Queue;
use rtt_target::rprintln;
pub use configs::{CONFIG_RESET, CONFIG_SELF_TEST_DISABLE, CONFIG_SELF_TEST_ENABLE, CONFIG_STREAMING, CONFIG_WAKEUP_LSM6DSOX};
use crate::registers::MainReg::{Ctrl1Xl, Ctrl2G};

/// Trait alias to support both I2c<SevenBitAddress> and I2c without address mode.
pub trait CompatibleI2c<E>: I2c<Error = E> {}
impl<T, E> CompatibleI2c<E> for T where T: I2c<Error = E> {}

pub const DEFAULT_ADDRESS: u8 = 0x6A;

struct TypedDebugWrapper<'a, T: ?Sized>(&'a T);

impl<T: Debug> Debug for TypedDebugWrapper<'_, T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}::{:?}", core::any::type_name::<T>(), self.0)
    }
}

trait TypedDebug: Debug {
    fn typed_debug(&self) -> TypedDebugWrapper<'_, Self> {
        TypedDebugWrapper(self)
    }
}

impl<T: ?Sized + Debug> TypedDebug for T {}


pub struct Lsm6dsox<I2C, E> {
    i2c: I2C,
    address: u8,
    _error: core::marker::PhantomData<E>,
}

impl<I2C, E> Lsm6dsox<I2C, E> {
    pub fn i2c(&mut self) -> &mut I2C {
        &mut self.i2c
    }
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidDevice,
    UnexpectedFifoTag(u8),
    ResetTimeout,
}

pub trait Lsm6dsoxInterface {
    type Error;

    fn write_register(&mut self, addr: u8, reg: u8, data: u8) -> Result<(), Self::Error>;
    fn read_register(&mut self, addr: u8, reg: u8) -> Result<u8, Self::Error>;
    fn read_data(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    fn apply_config(&mut self, config: &[(MainReg, u8)]) -> Result<(), Self::Error>;
    fn read_fifo_entry(&mut self) -> Result<(u8, [u8; 6]), Self::Error>;
    fn read_fifo_bytes(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> Lsm6dsox<I2C, E>
where
    I2C: CompatibleI2c<E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            _error: core::marker::PhantomData,
        }
    }

    pub fn default(i2c: I2C) -> Self {
        Self::new(i2c, DEFAULT_ADDRESS)
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    pub fn who_am_i(&mut self) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.address, &[MainReg::WhoAmI as u8], &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.apply_config(&CONFIG_RESET)?;
        self.wait_for_reset_complete().map_err(Error::I2c)?;
        Ok(())
    }

    fn wait_for_reset_complete(&mut self) -> Result<(), E> {
        loop {
            let mut buf = [0u8];
            self.i2c.write_read(self.address, &[MainReg::Ctrl3C as u8], &mut buf)?;
            if buf[0] & 0x01 == 0 {
                break;
            }
        }
        Ok(())
    }

    // pub fn do_soft_reset(&mut self) -> Result<(), Error<E>> {
    //     self.write_reg(MainReg::Ctrl3C.into(), Ctrl3CFlags::SW_RESET.bits())?;
    //     self.wait_for_reset_complete().map_err(Error::I2c)?;
    //     Ok(())
    // }

    pub fn do_soft_reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.write_reg(MainReg::Ctrl3C.into(), Ctrl3CFlags::SW_RESET.bits())?;

        // Wait a bit to allow device to reset
        for _ in 0..50 {
            delay.delay_ns(2_000_u32); // total max ~100ms wait

            let mut buf = [0u8];
            if self.i2c.write_read(self.address, &[MainReg::Ctrl3C as u8], &mut buf).is_ok() {
                if buf[0] & Ctrl3CFlags::SW_RESET.bits() == 0 {
                    return Ok(());
                }
            }
        }

        Err(Error::ResetTimeout)
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg, val])
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Accepts any register type that implements the `Register` trait
    pub fn apply_config<R>(&mut self, config: &[RegConfig<R>]) -> Result<(), Error<E>>
    where
        R: Register + NamedRegister + Copy,
    {
        for entry in config {
            let addr = entry.reg.addr();
            match entry.op {
                RegOp::Write => {
                    rprintln!("write_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, entry.value);
                    self.write_reg(addr, entry.value)?
                },
                RegOp::Read => {
                    let data = self.read_reg(addr)?;
                    rprintln!("read_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, data);
                }
            }
        }
        Ok(())
    }

    pub fn apply_any_config(&mut self, config: &[AnyRegConfig]) -> Result<(), Error<E>> {
        for entry in config {
            let addr = entry.reg.addr();
            match entry.op {
                RegOp::Write => {
                    rprintln!("write_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, entry.value);
                    self.write_reg(addr, entry.value)?;
                }
                RegOp::Read => {
                    let data = self.read_reg(addr)?;
                    rprintln!("read_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, data);
                }
            }
        }
        Ok(())
    }
    pub fn get_gyro_settling(&mut self) -> u8 {
        let status:u8 = self.read_reg(MainReg::UiStatusRegOis as u8).unwrap();
        let gyro_settling:u8 = status & 0x04;
        gyro_settling
    }

    pub fn read_bytes(&mut self, start_reg: u8, buffer: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.address, &[start_reg], buffer)
            .map_err(Error::I2c)
    }

    pub fn read_temperature(&mut self) -> Result<Option<f32>, Error<E>> {
        use core::convert::TryInto;

        // Optional: check if sensors are active
        let ctrl1_xl = self.read_reg(0x10)?; // CTRL1_XL
        let ctrl2_g  = self.read_reg(0x11)?; // CTRL2_G

        let accel_on = (ctrl1_xl & CTRL1_XL_ODR_MASK) != 0; // ODR_XL
        let gyro_on  = (ctrl2_g  & CTRL2_G_ODR_MASK) != 0; // ODR_G

        if !(accel_on || gyro_on) {
            return Ok(None); // Temperature not valid if no sensor is active
        }

        let temp_l = self.read_reg(0x20)?; // OUT_TEMP_L
        let temp_h = self.read_reg(0x21)?; // OUT_TEMP_H

        let raw = i16::from_le_bytes([temp_l, temp_h]);
        let temp_c = (raw as f32) / 256.0 + 25.0;

        Ok(Some(temp_c))
    }

    pub fn get_linear_acc_sensitivty(&mut self) -> Result<Option<f32>, Error<E>> {
        use core::convert::TryInto;

        // Optional: check if sensors are active
        let ctrl1_xl = self.read_reg(Ctrl1Xl as u8)?; // CTRL1_XL

        let accel_on = (ctrl1_xl & CTRL1_XL_ODR_MASK) != 0; // ODR_XL

        if !(accel_on) {
            return Ok(None); // Temperature not valid if no sensor is active
        }

        let fs = (ctrl1_xl & CTRL1_XL_FS_MASK) >> CTRL1_XL_FS_LOC;
        match fs {
            0x00 => Ok(Some(0.061)),
            0x01 => Ok(Some(0.122)),
            0x11 => Ok(Some(0.244)),
            0x10 => Ok(Some(0.488)),
            _ => Ok(None),
        }
    }

    pub fn get_angular_rate_sensitivty(&mut self) -> Result<Option<f32>, Error<E>> {
        use core::convert::TryInto;

        // Optional: check if sensors are active
        let ctrl2_g = self.read_reg(Ctrl2G as u8)?; // CTRL1_XL

        let gyro_on = (ctrl2_g & CTRL2_G_ODR_MASK) != 0; // ODR_XL

        if !(gyro_on) {
            return Ok(None); // Temperature not valid if no sensor is active
        }

        let fs = (ctrl2_g & CTRL2_G_FS_MASK) >> CTRL2_G_FS_LOC;
        match fs {
            0x00 => Ok(Some(0.00875)),
            0x01 => Ok(Some(0.004375)),
            0x02 => Ok(Some(0.0175)),
            0x04 => Ok(Some(0.035)),
            0x06 => Ok(Some(0.07)),
            _ => Ok(None),
        }
    }

    pub fn read_fifo_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<E>> {
        const FIFO_DATA_OUT_TAG: u8 = 0x78; // Datasheet: FIFO_DATA_OUT_TAG register
        self.read_bytes(FIFO_DATA_OUT_TAG, buf)
    }

    pub fn read_next_sample(&mut self) -> Result<Option<SensorSample>, E> {
        // Parses FIFO and returns the next complete sample, or None if incomplete
        Ok(None)
    }

    pub fn select_register_bank(&mut self, mode: FuncCfgAccessMode) -> Result<(), Error<E>> {
        use FuncCfgAccessFlags as Flags;

        let flags = match mode {
            FuncCfgAccessMode::User => Flags::empty(),
            FuncCfgAccessMode::EmbFuncCfg => Flags::EMBFUNC_CFG_ACCESS,
            FuncCfgAccessMode::SensorHub  => Flags::SHUB_REG_ACCESS,
            FuncCfgAccessMode::OisCtrl    => Flags::OIS_CTRL_ACCESS,
        };

        self.write_reg(MainReg::FuncCfgAccess.into(), flags.bits())
    }

    pub fn dump_config<R>(&mut self, regs: &[R]) -> Result<(), Error<E>>
    where
        R: NamedRegister + Copy,
    {
        use rtt_target::rprintln;

        fn show(label: &str, reg: u8, val: Result<u8, impl core::fmt::Debug>) {
            match val {
                Ok(v) => rprintln!("{:<21}({:#04x}): 0x{:02X} ({:>3}) 0b{:08b}", label, reg, v, v, v),
                Err(e) => rprintln!("{:<16}: Error: {:?}", label, e),
            }
        }

        for reg in regs {
            let label = reg.name();
            let addr = reg.addr();
            show(label, addr, self.read_reg(addr));
        }

        Ok(())
    }

    pub fn dump_reg_config(&mut self, reg: &MainReg) -> Result<(), Error<E>> {
        use rtt_target::rprintln;

        fn show(label: &str, reg: u8, val: Result<u8, impl core::fmt::Debug>) {
            match val {
                Ok(v) => rprintln!("{:<21}({:#04x}): 0x{:02X} ({})", label, reg, v, v),
                Err(e) => rprintln!("{:<16}: Error: {:?}", label, e),
            }
        }
        let label = reg.name();       // &'static str
        let reg = *reg as u8;       // u8 value
        show(label, reg, self.read_reg(reg));

        Ok(())
    }

    /// Flush all FIFO contents by setting bypass mode and draining residual samples.
    pub fn flush_fifo(&mut self) -> Result<(), Error<E>> {
        // use crate::registers::{MainReg, FifoStatus1, FifoStatus2, FifoDataOutTag};

        // Set FIFO mode to BYPASS (0b000) — clears internal FIFO buffer
        self.write_reg(MainReg::FifoCtrl4.into(), 0x00)?;

        // Optional small delay if needed for hardware to process flush
        // (you can add a delay method to your driver or use a timer externally)
        // self.delay.delay_ms(10);

        // Read until FIFO reports empty
        loop {
            let status1 = self.read_reg(MainReg::FifoStatus1.into())? as u16;
            let status2 = self.read_reg(MainReg::FifoStatus2.into())? as u16;
            let samples = ((status2 & 0x03) << 8) | status1;

            if samples == 0 {
                break;
            }

            // Discard one full FIFO frame (tag + 6 bytes)
            let mut buf = [0u8; 7];
            self.read_fifo_bytes(&mut buf)?;
        }

        Ok(())
    }

    // pub fn wait_for_sensorhub_ready<D: DelayNs, E>(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
    //     use crate::registers::MainReg::StatusMasterMainpage;
    //
    //     // STATUS_MASTER_MAINPAGE = 0x39
    //     loop {
    //         let status = self.read_register(StatusMasterMainpage)?;
    //         if status & 0x01 != 0 {
    //             return Ok(()); // SHUB busy cleared
    //         }
    //         delay.delay_ms(1);
    //     }
    // }

}

#[derive(Debug, Clone, Copy)]
pub struct AccelSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct GyroSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct MagSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

pub struct SensorSample {
    pub accel: Option<[i16; 3]>,
    pub gyro: Option<[i16; 3]>,
    pub mag: Option<[i16; 3]>,
    pub timestamp: Option<u32>,
}

pub trait IMUStreamDriver {
    type Error;

    fn fifo_samples_available(&mut self) -> Result<u16, Self::Error>;
    fn read_fifo_sample(&mut self) -> Result<FifoSample, Self::Error>;
    fn dump_fifo(&mut self);

}

impl<I2C, E> IMUStreamDriver for Lsm6dsox<I2C, E>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: core::fmt::Debug,  // Added Debug trait bound
{
    type Error = Error<E>;

    fn fifo_samples_available(&mut self) -> Result<u16, Self::Error> {
        let fifo_status1 = self.read_reg(MainReg::FifoStatus1.into())?;
        let fifo_status2 = self.read_reg(MainReg::FifoStatus2.into())?;
        let sample_count = (((fifo_status2 & 0x03) as u16) << 8) | (fifo_status1 as u8) as u16;
        Ok(sample_count)
    }

    fn read_fifo_sample(&mut self) -> Result<FifoSample, Self::Error> {
        let mut buf = [0u8; 7];
        self.read_fifo_bytes(&mut buf)?;

        // rprintln!("read_fifo_sample {:?}", buf);

        let tag_val = (buf[0] & 0xf8) >> 3;

        let tag = FifoTags::try_from(tag_val).map_err(|_| Error::UnexpectedFifoTag(tag_val))?;
        rprintln!("read_fifo_sample tag:{:#02X}({})", tag, tag.name());

        let sample = match tag {
            FifoTags::AccelerometerNc => FifoSample::Accel([
                i16::from_le_bytes([buf[1], buf[2]]),
                i16::from_le_bytes([buf[3], buf[4]]),
                i16::from_le_bytes([buf[5], buf[6]]),
            ]),
            FifoTags::GyroscopeNc => FifoSample::Gyro([
                i16::from_le_bytes([buf[1], buf[2]]),
                i16::from_le_bytes([buf[3], buf[4]]),
                i16::from_le_bytes([buf[5], buf[6]]),
            ]),
            FifoTags::SensorHubSlave0 => FifoSample::Mag([
                i16::from_le_bytes([buf[1], buf[2]]),
                i16::from_le_bytes([buf[3], buf[4]]),
                i16::from_le_bytes([buf[5], buf[6]]),
            ]),
            FifoTags::Temperature => FifoSample::Temperature([
                i16::from_le_bytes([buf[1], buf[2]]), // Convert i16 to f32
            ]),
            FifoTags::SensorHubNack => FifoSample::Unknown(tag_val),
            // Add more tags as needed
            _ => return Err(Error::UnexpectedFifoTag(tag_val)),
        };

        Ok(sample)
    }

    fn dump_fifo(&mut self)  {
        let count = self.fifo_samples_available().unwrap_or(0);
        rprintln!("FIFO samples available: {}", count);
        for _ in 0..count {
            if let Ok(sample) = self.read_fifo_sample() {
                match sample {
                    FifoSample::Accel(accel_data) => {
                        // info!("Accel: {:?}", accel_data);
                        rprintln!("Accel: {:?}", accel_data);
                    }
                    FifoSample::Gyro(gyro_data) => {
                        // info!("Gyro: {:?}", gyro_data);
                        rprintln!("Gyro: {:?}", gyro_data);
                    }
                    FifoSample::Mag(mag_data) => {
                        // info!("Mag: {:?}", mag_data);
                        rprintln!("Mag: {:?}", mag_data);
                    }
                    FifoSample::Temperature(temp_data) => {
                        // info!("Temperature: {:?}", temp_data);
                        rprintln!("Temperature: {:?}", temp_data);
                    }
                    FifoSample::Unknown(tag) => {
                        // warn!("Unknown sample type: {}", tag);
                        rprintln!("Unknown sample type: {:#02X}", tag);
                    }
                }
            }
        }
    }

}

pub struct StreamBuffer<const N: usize> {
    queue: Queue<FifoSample, N>,
}

impl<const N: usize> StreamBuffer<N> {
    pub fn new() -> Self {
        Self {
            queue: Queue::new(),
        }
    }

    pub fn push(&mut self, sample: FifoSample) {
        let _ = self.queue.enqueue(sample); // silently drop if full
    }

    pub fn pop(&mut self) -> Option<FifoSample> {
        self.queue.dequeue()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }
}