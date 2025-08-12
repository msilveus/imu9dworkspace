use crate::registers::*;

pub const CONFIG_WAKEUP_LIS3MDL: &[RegConfig<MagReg>] = &[
    RegConfig {
        reg: MagReg::CtrlReg1,
        value: (MagOdr::Hz40 as u8) << MAG_ODR_LOC | (MagOM::HighPowerMode as u8) << MAG_OM_LOC, // 80Hz
        op: RegOp::Write,
    },
    RegConfig {
        reg: MagReg::CtrlReg2,
        value: (MagFullScale::G4 as u8) << MAG_FS_LOC, // ±4 gauss
        op: RegOp::Write,
    },
    RegConfig {
        reg: MagReg::CtrlReg3,
        value: MagConvMode::Continuous as u8, // Continuous-conversion mode
        op: RegOp::Write,
    },
    RegConfig {
        reg: MagReg::CtrlReg4,
        value: (MagOMZ::HighPerformanceMode as u8) << MAG_OMZ_LOC | (MagBLE::LittleEndian as u8) << MAG_BLE_LOC, // Ultra-high performance Z
        op: RegOp::Write,
    },
    RegConfig {
        reg: MagReg::CtrlReg5,
        value: CtrlReg5Flags::BDU.bits(), // Update the output after both MSB and LSB are read
        op: RegOp::Write,
    },
    RegConfig {
        reg: MagReg::IntCfg,
        value: 0x00, // All interrupts disabled
        op: RegOp::Write,
    },
];

pub const CONFIG_RESET: &[RegConfig<MagReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MagReg::CtrlReg2,
        value: CtrlReg2Bitflags::SoftReset as u8, // Set SW_RESET bit
    },
];
