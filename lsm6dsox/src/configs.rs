#![allow(unused_imports)]

use crate::registers::*;
use crate::registers::FuncCfgAccessMode::SensorHub;

// Trigger software reset
pub const CONFIG_RESET: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl3C,
        value: 0x01, // Set SW_RESET bit
    },
];


pub const CONFIG_WAKEUP_LSM6DSOX: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl9Xl,
        value: Ctrl9XlFlags::I3C_DISABLE.bits(), // I3C_DISABLE
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::I3CBusAvb,
        value: I3CBusAvbFlags::I3C_Bus_Avb_Sel50u.bits(), // I3C_DISABLE
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl1Xl,
        value: (Odr::Hz104 as u8) << CTRL1_XL_ODR_LOC | (AccelFullScale::G2 as u8) << CTRL1_XL_FS_LOC,
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl2G,
        value: (Odr::Hz104 as u8) << CTRL2_G_ODR_LOC | (GyroFullScale::DPS125 as u8) << CTRL2_G_FS_125_LOC,
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl3C,
        value: Ctrl3CFlags::BDU.bits() | Ctrl3CFlags::IF_INC.bits(),
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl4C,
        value: Ctrl4CFlags::INT2_ON_INT1.bits() | Ctrl4CFlags::LPF1_SEL_G.bits(), // all interrupt signals in logic or on INT1 pin
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl5C,
        value: 0x00, // Normal mode
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl6C,
        value: 0x07, // 11.5 LPF bandwidth
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl7G,
        value: 0x00, // default
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl8Xl,
        value: 0x00, // Default
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl10C,
        value: 0x00, // No timestamp batching
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Int1Ctrl,
        value: Int1CtrlBitflags::Int1FifoTh as u8, // INT1_FIFO_TH
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Int2Ctrl,
        value: 0x00, // No Int2
    },
];

pub const CONFIG_STREAMING: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::FifoCtrl1,
        value: 15, // watermark level = 5 samples 6 bytes each of acc/gyro/mag
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::FifoCtrl2,
        value: 0x10, // ODRCHG_EN
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::FifoCtrl3,
        value: (FifoBatchOdr::Hz104 as u8) << FIFO_BATCH_ODR_XL_LOC | (FifoBatchOdr::Hz104 as u8) << FIFO_BATCH_ODR_G_LOC, // include both XL and GYRO in FIFO @ 194hz
    },
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::FifoCtrl4,
        value: (FifoMode::Continuous as u8) << FIFO_MODE_LOC | (FifoOdrT::Hz52 as u8) << FIFO_TEMP_ODR_LOC, // FIFO as continuous, temperature batch
    },
];

pub const CONFIG_INT_NOTIFICATION: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::TapCfg0,
        value: TapCfg0Flags::LIR.bits() | TapCfg0Flags::INT_CLR_ON_READ.bits(), // LSM6DSO_ALL_INT_LATCHED
    },
];

pub const CONFIG_SELF_TEST_ENABLE: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl5C,
        value: 0b00000110, // XL self-test positive + G self-test positive
    },
];

pub const CONFIG_SELF_TEST_DISABLE: &[RegConfig<MainReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: MainReg::Ctrl5C,
        value: 0x00, // Normal mode
    },
];

pub const CONFIG_SENSOR_HUB_LIS3MDL: &[RegConfig<SensorHubReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),  // enable SensorHubReg
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::Slv0Addr,
        value: 0x39, // LIS3MDL read address << 1 (0x1C << 1) + 1(Read)
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::Slv0SubAddr,
        value: 0x28, // OUTX_L_REG
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::Slv0Config,
        value: (ShubOdrRate::Hz104 as u8) << SHUB_ODR_LOC | (EnableFlags::Enabled as u8) << BATCH_EXT_SENS_0_EN_LOC | 6, // read 6 bytes
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::MasterConfig,
        value:  MasterConfigFlags::WRITE_ONCE.bits() |
            MasterConfigFlags::START_CONFIG.bits() |
            MasterConfigFlags::SHUB_PU_EN.bits() |
            MasterConfigAuxSensor::OneSensor as u8, // enable sensor hub master
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: FuncCfgAccessFlags::empty().bits(),  // enable SensorHubReg
    },
];

pub const CONFIG_SH_MASTER_RESET: &[RegConfig<SensorHubReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),  // enable SensorHubReg
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::MasterConfig,
        value: MasterConfigFlags::RST_MASTER_REGS.bits(), // set sensor hub master reset bit
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::MasterConfig,
        value: MasterConfigFlags::empty().bits(), // clear sensor hub master reset bit
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: 0, // disable SensorHubReg
    },
];

pub const CONFIG_EMB_FUNCS: &[RegConfig<EmbeddedFuncReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: EmbeddedFuncReg::FuncCfgAccess,
        value: FuncCfgAccessFlags::EMBFUNC_CFG_ACCESS.bits(),  // enable SensorHubReg
    },
    RegConfig {
        op: RegOp::Write,
        reg: EmbeddedFuncReg::PageRw,
        value: PageRwFlags::EMB_FUNC_LIR.bits(), // embedded functions interrupt request latched
    },
    RegConfig {
        op: RegOp::Write,
        reg: EmbeddedFuncReg::FuncCfgAccess,
        value: 0, // disable EmbeddedFuncReg
    },
];

pub const CONFIG_SENSOR_HUB_LIS3MDL_MIXED: &[AnyRegConfig] = &[
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Ctrl6C),
        value: 0x07, // 11.5 LPF bandwidth
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Ctrl9Xl),
        value: Ctrl9XlFlags::I3C_DISABLE.bits(), // I3C_DISABLE
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FifoCtrl1),
        value: 15, // watermark level = 5 samples 6 bytes each of acc/gyro/mag
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FifoCtrl4),
        value: (FifoMode::Bypass as u8) << FIFO_MODE_LOC | (FifoOdrT::Hz52 as u8) << FIFO_TEMP_ODR_LOC,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::TapCfg0),
        value: TapCfg0Flags::LIR.bits() | TapCfg0Flags::INT_CLR_ON_READ.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Int1Ctrl),
        value: Int1CtrlBitflags::Int1FifoTh as u8,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::SensorHub(SensorHubReg::Slv0Config),
        value: (ShubOdrRate::Hz104 as u8) << SHUB_ODR_LOC | (EnableFlags::Enabled as u8) << BATCH_EXT_SENS_0_EN_LOC | 6,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::empty().bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FifoCtrl3),
        value: (FifoBatchOdr::Hz104 as u8) << FIFO_BATCH_ODR_XL_LOC | (FifoBatchOdr::Hz104 as u8) << FIFO_BATCH_ODR_G_LOC,
    },
    // AnyRegConfig {
    //     op: RegOp::Write,
    //     reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
    //     value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),
    // },
    // AnyRegConfig {
    //     op: RegOp::Write,
    //     reg: UnifiedRegister::SensorHub(SensorHubReg::Slv0Addr),
    //     value: 0x39,
    // },
    // AnyRegConfig {
    //     op: RegOp::Write,
    //     reg: UnifiedRegister::SensorHub(SensorHubReg::Slv1SubAddr),
    //     value: 0x28,
    // },
    // AnyRegConfig {
    //     op: RegOp::Write,
    //     reg: UnifiedRegister::SensorHub(SensorHubReg::MasterConfig),
    //     value:  MasterConfigAuxSensor::OneSensor as u8 |
    //             MasterConfigFlags::SHUB_PU_EN.bits() |
    //             MasterConfigFlags::START_CONFIG.bits() |
    //             MasterConfigFlags::WRITE_ONCE.bits(),
    // },
    // AnyRegConfig {
    //     op: RegOp::Write,
    //     reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
    //     value: FuncCfgAccessFlags::empty().bits(),
    // },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Ctrl1Xl),
        value: (Odr::Hz52 as u8) << CTRL1_XL_ODR_LOC | (AccelFullScale::G2 as u8) << CTRL1_XL_FS_LOC,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Ctrl2G),
        value: (Odr::Hz52 as u8) << CTRL2_G_ODR_LOC | (GyroFullScale::DPS125 as u8) << CTRL2_G_FS_125_LOC,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::Ctrl3C),
        value: Ctrl3CFlags::BDU.bits() | Ctrl3CFlags::IF_INC.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::EMBFUNC_CFG_ACCESS.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::EmbeddedFunc(EmbeddedFuncReg::EmbFuncOdrCfgB),
        value: (FsmOdrCfgB::Hz104 as u8) << FSM_ODR_CFG_B_LOC | 0x43, // 0x43 mandatory bits
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::empty().bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::SensorHub(SensorHubReg::Slv0Addr),
        value: 0x39,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::SensorHub(SensorHubReg::Slv1SubAddr),
        value: 0x28,
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::SensorHub(SensorHubReg::MasterConfig),
        value:  MasterConfigAuxSensor::OneSensor as u8 |
            // MasterConfigFlags::START_CONFIG.bits() |
            MasterConfigFlags::WRITE_ONCE.bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FuncCfgAccess),
        value: FuncCfgAccessFlags::empty().bits(),
    },
    AnyRegConfig {
        op: RegOp::Write,
        reg: UnifiedRegister::Main(MainReg::FifoCtrl4),
        value: (FifoMode::Continuous as u8) << FIFO_MODE_LOC | (FifoOdrT::Hz52 as u8) << FIFO_TEMP_ODR_LOC,
    },
];

pub const CONFIG_SH_MASTER_ON: &[RegConfig<SensorHubReg>] = &[
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: FuncCfgAccessFlags::SHUB_REG_ACCESS.bits(),  // enable SensorHubReg
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::MasterConfig,
        value:  MasterConfigAuxSensor::OneSensor as u8 |
            // MasterConfigFlags::START_CONFIG.bits() |
            MasterConfigFlags::WRITE_ONCE.bits() |
            MasterConfigFlags::MASTER_ON.bits(),
    },
    RegConfig {
        op: RegOp::Write,
        reg: SensorHubReg::FuncCfgAccess,
        value: 0, // disable SensorHubReg
    },
];
