use core::cell::RefCell;
use critical_section::Mutex;

use embedded_hal::i2c::{self, ErrorType, I2c, Operation, SevenBitAddress};
use stm32f4xx_hal::i2c::{Error, I2c as HalI2c};
use stm32f4xx_hal::pac::I2C1;

static I2C_BUS: Mutex<RefCell<Option<HalI2c<I2C1>>>> = Mutex::new(RefCell::new(None));

pub struct SharedI2c;

impl SharedI2c {
    pub fn init(i2c: HalI2c<I2C1>) {
        critical_section::with(|cs| {
            *I2C_BUS.borrow(cs).borrow_mut() = Some(i2c);
        });
    }

    fn with<R>(f: impl FnOnce(&mut HalI2c<I2C1>) -> R) -> R {
        critical_section::with(|cs| {
            let mut borrow = I2C_BUS.borrow(cs).borrow_mut();
            let i2c = borrow.as_mut().expect("Shared I2C not initialized");
            f(i2c)
        })
    }
}

// Required by the embedded-hal I2c trait
impl ErrorType for SharedI2c {
    type Error = Error;
}

impl I2c for SharedI2c {
    fn read(&mut self, addr: SevenBitAddress, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::with(|i2c| i2c.read(addr, buffer))
    }

    fn write(&mut self, addr: SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::with(|i2c| i2c.write(addr, bytes))
    }

    fn write_read(
        &mut self,
        addr: SevenBitAddress,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::with(|i2c| i2c.write_read(addr, bytes, buffer))
    }

    // Dummy implementation to satisfy the trait — HAL doesn't support this natively.
    fn transaction(
        &mut self,
        _addr: SevenBitAddress,
        _operations: &mut [Operation],
    ) -> Result<(), Self::Error> {
        Err(Error::Bus) // or Error::Other, or anything non-panic
    }
}
