#![allow(unused_imports)]
use core::convert::TryFrom;
use bitflags::bitflags;

macro_rules! registers {
    (
        $enum_name:ident, $slice_name:ident {
            $($name:ident = $val:expr),* $(,)?
        }
    ) => {
        #[repr(u8)]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        pub enum $enum_name {
            $($name = $val),*
        }

        pub const $slice_name: &[$enum_name] = &[
            $($enum_name::$name),*
        ];

        impl $enum_name {
            pub fn name(&self) -> &'static str {
                match self {
                    $($enum_name::$name => stringify!($name),)*
                }
            }
        }

        impl Register for $enum_name {
            fn addr(self) -> u8 {
                self as u8
            }
        }

        impl NamedRegister for $enum_name {
            fn name(&self) -> &'static str {
                self.name()
            }
        }

        impl From<$enum_name> for u8 {
            fn from(r: $enum_name) -> u8 {
                r as u8
            }
        }
    };
}

#[derive(Clone, Copy, Debug)]
pub enum RegOp {
    Read,
    Write
}

pub trait NamedRegister: Register {
    fn name(&self) -> &'static str;
}

pub trait Register: Copy {
    fn addr(self) -> u8;
}

pub struct RegConfig<R: Register> {
    pub op: RegOp,
    pub reg: R,
    pub value: u8,
}

registers! {
    MagReg, MAG_REGS {
        OffsetXRegLM = 0x05,
        OffsetXRegHM = 0x06,
        OffsetYRegLM = 0x07,
        OffsetYRegHM = 0x08,
        OffsetZRegLM = 0x09,
        OffsetZRegHM = 0x0A,
        WhoAmI = 0x0F,
        CtrlReg1 = 0x20,
        CtrlReg2 = 0x21,
        CtrlReg3 = 0x22,
        CtrlReg4 = 0x23,
        CtrlReg5 = 0x24,
        StatusReg = 0x27,
        OutXL = 0x28,
        OutXH = 0x29,
        OutYL = 0x2A,
        OutYH = 0x2B,
        OutZL = 0x2C,
        OutZH = 0x2D,
        TempOutL = 0x2E,
        TempOutH = 0x2F,
        IntCfg = 0x30,
        IntSrc = 0x31,
        IntThsL = 0x32,
        IntThsH = 0x33,
    }
}

bitflags::bitflags! {
    pub struct StatusFlags: u8 {
        const ZYXOR = 1 << 7;
        const ZOR   = 1 << 6;
        const YOR   = 1 << 5;
        const XOR   = 1 << 4;
        const ZYXDA = 1 << 3;
        const ZDA   = 1 << 2;
        const YDA   = 1 << 1;
        const XDA   = 1 << 0;
    }
}

/* CTRL_REG1
 * B7   B6   B5   B4   B3   B2   B1   B0
 * TEMP OM1  OM0  DO2  DO1  DO0  FODR ST
*/
#[repr(u8)]
pub enum CtrlReg1Bitflags {
    SelfTest        = 1 << 0,
    FastOdr         = 1 << 1,
    TemperatureEn   = 1 << 7,
}

pub const MAG_ODR_LOC: u8 = 2;
#[repr(u8)]
pub enum MagOdr {
    Hz0_625 = 0,
    Hz1_25  = 1,
    Hz2_5   = 2,
    Hz5     = 3,
    Hz10    = 4,
    Hz20    = 5,
    Hz40    = 6,
    Hz80    = 7,
}

pub const MAG_OM_LOC: u8 = 5;
#[repr(u8)]
pub enum MagOM {
    LowPowerMode    = 0,
    MedPowerMode    = 1,
    HighPowerMode   = 2,
    UltraPowerMode  = 3,
}

/* CTRL_REG2
 * B7   B6   B5   B4   B3   B2   B1   B0
 * 0    FS1  FS0  0    R    S    0    0
 *                     E    O
 *                     B    F
 *                     O    T
 *                     O    R
 *                     T    S
 *                          T
*/
#[repr(u8)]
pub enum CtrlReg2Bitflags {
    SoftReset   = 1 << 2,
    Reboot      = 1 << 3,
}

pub const MAG_FS_LOC: u8 = 5;
#[repr(u8)]
pub enum MagFullScale {
    G4  = 0,
    G8  = 1,
    G12 = 2,
    G16 = 3,
}

/* CTRL_REG3
 * B7   B6   B5   B4   B3   B2   B1   B0
 * 0    0    LP   0    0    SIM  MD1  MD0
*/
#[repr(u8)]
pub enum CtrlReg3Bitflags {
    PowerDown       = 1 << 1,
    LowPowerMode    = 1 << 5,
}

/* CTRL_REG4
 * B7   B6   B5   B4   B3   B2   B1   B0
 * 0    0    0    0    OMZ1 OMZ0 BLE  0
*/
pub const MAG_OMZ_LOC: u8 = 2;
#[repr(u8)]
pub enum MagOMZ {
    LowPowerMode            = 0,
    MedPerformanceMode      = 1,
    HighPerformanceMode     = 2,
    UltraPerformanceMode    = 3,
}

pub const MAG_BLE_LOC: u8 = 1;
#[repr(u8)]
pub enum MagBLE {
    LittleEndian    = 0,
    BigEndian       = 1,
}


#[repr(u8)]
pub enum MagConvMode { // Bit 0
    Continuous  = 0,
    SingleConv  = 1,
}

pub const MAG_SIM_LOC: u8 = 2;
#[repr(u8)]
pub enum MagSimMode {
    Intf4Wire  = 0,
    Intf3Wire  = 1,
}

/* CTRL_REG5
 * B7   B6   B5   B4   B3   B2   B1   B0
 * F    B    0    0    0    0    0    0
 * A    D
 * S    U
 * T
 *
 * R
 * E
 * A
 * D
*/
bitflags::bitflags! {
    pub struct CtrlReg5Flags: u8 {
        const FAST_READ = 1 << 7;
        const BDU       = 1 << 6;
    }
}

pub const MAG_BDU_LOC: u8 = 6;
#[repr(u8)]
pub enum MagBDU {
    Continuous  = 0,
    BlockMode   = 1,
}

pub const MAG_FASTREAD_LOC: u8 = 7;
#[repr(u8)]
pub enum MagFastRead {
    Disabled    = 0,
    Enabled     = 1,
}

