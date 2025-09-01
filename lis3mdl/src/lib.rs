#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

pub mod registers;
pub mod configs;

use core::fmt::{Debug, Formatter};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use log::{Log, Level, Metadata, Record, LevelFilter,trace, debug, info, warn, error, set_logger, set_max_level};

// use rtt_target::debug;

use registers::*;

pub use configs::*;

/// Trait alias to support both I2c<SevenBitAddress> and I2c without address mode.
pub trait CompatibleI2c<E>: I2c<Error = E> {}
impl<T, E> CompatibleI2c<E> for T where T: I2c<Error = E> {}

pub const DEFAULT_ADDRESS: u8 = 0x1C;

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


pub struct Lis3mdl<I2C, E> {
    i2c: I2C,
    address: u8,
    _error: core::marker::PhantomData<E>,
}

impl<I2C, E> Lis3mdl<I2C, E> {
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

impl<I2C, E> Lis3mdl<I2C, E>
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
            .write_read(self.address, &[MagReg::WhoAmI as u8], &mut buf)
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
            self.i2c.write_read(self.address, &[MagReg::CtrlReg2 as u8], &mut buf)?;
            if buf[0] & 0x01 == 0 {
                break;
            }
        }
        Ok(())
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

    pub fn read_bytes(&mut self, start_reg: u8, buffer: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.address, &[start_reg], buffer)
            .map_err(Error::I2c)
    }

    /// Accepts any register type that implements the `Register` trait
    pub fn apply_config<R>(&mut self, config: &[RegConfig<R>]) -> Result<(), Error<E>>
    where
        R: Register + NamedRegister + Copy,
    {
        for entry in config {
            let addr = entry.reg.addr(); // ← Use your trait method here
            match entry.op {
                RegOp::Write => {
                    debug!("write_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, entry.value);
                    self.write_reg(addr, entry.value)?
                },
                RegOp::Read => {
                    let data = self.read_reg(addr)?;
                    debug!("read_reg {:<21}({:#04X}) = {:#04x}", entry.reg.name(), addr, data);
                }
            }
        }
        Ok(())
    }

    pub fn dump_config<R>(&mut self, regs: &[R]) -> Result<(), Error<E>>
    where
        R: NamedRegister + Copy,
    {
        fn show(label: &str, reg: u8, val: Result<u8, impl core::fmt::Debug>) {
            match val {
                Ok(v) => debug!("{:<21}({:#04x}): 0x{:02X} ({:>3}) 0b{:08b}", label, reg, v, v, v),
                Err(e) => debug!("{:<16}: Error: {:?}", label, e),
            }
        }

        for reg in regs {
            let label = reg.name();
            let addr = reg.addr();
            show(label, addr, self.read_reg(addr));
        }

        Ok(())
    }

}
