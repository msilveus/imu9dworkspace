#![allow(unused_imports)]
use core::convert::TryFrom;
use bitflags::bitflags;

use paste::paste;

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

        paste! {
            #[allow(non_snake_case)]
            pub fn [<$enum_name _Stringify_From_u8>](value: u8) -> Option<&'static str> {
                $slice_name.iter().find(|r| r.addr() == value).map(|r| r.name())
            }
        }
    };
}

macro_rules! impl_register {
    ($reg:ty) => {
        impl From<$reg> for u8 {
            fn from(r: $reg) -> u8 {
                r as u8
            }
        }

        impl Register for $reg {
            fn addr(self) -> u8 {
                self as u8
            }
        }
    };
}

#[derive(Clone, Copy, Debug)]
pub enum RegOp {
    Read,
    Write
}

#[derive(Clone, Copy, Debug)]
pub enum EnableFlags {
    Disabled = 0,
    Enabled = 1,
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

pub struct AnyRegConfig {
    pub op: RegOp,
    pub reg: UnifiedRegister,
    pub value: u8,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UnifiedRegister {
    Main(MainReg),
    SensorHub(SensorHubReg),
    EmbeddedFunc(EmbeddedFuncReg),
}

impl Register for UnifiedRegister {
    fn addr(self) -> u8 {
        match self {
            UnifiedRegister::Main(r) => r.addr(),
            UnifiedRegister::SensorHub(r) => r.addr(),
            UnifiedRegister::EmbeddedFunc(r) => r.addr(),
        }
    }
}

impl NamedRegister for UnifiedRegister {
    fn name(&self) -> &'static str {
        match self {
            UnifiedRegister::Main(r) => r.name(),
            UnifiedRegister::SensorHub(r) => r.name(),
            UnifiedRegister::EmbeddedFunc(r) => r.name(),
        }
    }
}

registers! {
    MainReg, MAIN_REGS {
        FuncCfgAccess           = 0x01,
        PinCtrl                 = 0x02,
        S4STphL                 = 0x04,
        S4STphH                 = 0x05,
        S4SRr                   = 0x06,
        FifoCtrl1               = 0x07,
        FifoCtrl2               = 0x08,
        FifoCtrl3               = 0x09,
        FifoCtrl4               = 0x0A,
        CounterBdrReg1          = 0x0B,
        CounterBdrReg2          = 0x0C,
        Int1Ctrl                = 0x0D,
        Int2Ctrl                = 0x0E,
        WhoAmI                  = 0x0F,
        Ctrl1Xl                 = 0x10,
        Ctrl2G                  = 0x11,
        Ctrl3C                  = 0x12,
        Ctrl4C                  = 0x13,
        Ctrl5C                  = 0x14,
        Ctrl6C                  = 0x15,
        Ctrl7G                  = 0x16,
        Ctrl8Xl                 = 0x17,
        Ctrl9Xl                 = 0x18,
        Ctrl10C                 = 0x19,
        AllIntSrc               = 0x1A,
        WakeUpSrc               = 0x1B,
        TapSrc                  = 0x1C,
        D6DSrc                  = 0x1D,
        StatusReg               = 0x1E,
        OutTempL                = 0x20,
        OutTempH                = 0x21,
        OutxLG                  = 0x22,
        OutxHG                  = 0x23,
        OutyLG                  = 0x24,
        OutyHG                  = 0x25,
        OutzLG                  = 0x26,
        OutzHG                  = 0x27,
        OutxLA                  = 0x28,
        OutxHA                  = 0x29,
        OutyLA                  = 0x2A,
        OutyHA                  = 0x2B,
        OutzLA                  = 0x2C,
        OutzHA                  = 0x2D,
        EmbFuncStatusMainpage   = 0x35,
        FsmStatusAMainpage      = 0x36,
        FsmStatusBMainpage      = 0x37,
        MlcStatusMainpage       = 0x38,
        StatusMasterMainpage    = 0x39,
        FifoStatus1             = 0x3A,
        FifoStatus2             = 0x3B,
        Timestamp0              = 0x40,
        Timestamp1              = 0x41,
        Timestamp2              = 0x42,
        Timestamp3              = 0x43,
        UiStatusRegOis          = 0x49,
        UiOutxLGOis             = 0x4A,
        UiOutxHGOis             = 0x4B,
        UiOutyLGOis             = 0x4C,
        UiOutyHGOis             = 0x4D,
        UiOutzLGOis             = 0x4E,
        UiOutzHGOis             = 0x4F,
        UiOutxLAOis             = 0x50,
        UiOutxHAOis             = 0x51,
        UiOutyLAOis             = 0x52,
        UiOutyHAOis             = 0x53,
        UiOutzLAOis             = 0x54,
        UiOutzHAOis             = 0x55,
        TapCfg0                 = 0x56,
        TapCfg1                 = 0x57,
        TapCfg2                 = 0x58,
        TapThs6D                = 0x59,
        IntDur2                 = 0x5A,
        WakeUpThs               = 0x5B,
        WakeUpDur               = 0x5C,
        FreeFall                = 0x5D,
        Md1Cfg                  = 0x5E,
        Md2Cfg                  = 0x5F,
        S4SStCmdCode            = 0x60,
        S4SDtReg                = 0x61,
        I3CBusAvb               = 0x62,
        InternalFreqFine        = 0x63,
        UiIntOis                = 0x6F,
        UiCtrl1Ois              = 0x70,
        UiCtrl2Ois              = 0x71,
        UiCtrl3Ois              = 0x72,
        XOfsUsr                 = 0x73,
        YOfsUsr                 = 0x74,
        ZOfsUsr                 = 0x75,
        FifoDataOutTag          = 0x78,
        FifoDataOutXL           = 0x79,
        FifoDataOutXH           = 0x7A,
        FifoDataOutYL           = 0x7B,
        FifoDataOutYH           = 0x7C,
        FifoDataOutZL           = 0x7D,
        FifoDataOutZH           = 0x7E
    }
}

bitflags::bitflags! {
    pub struct Ctrl4CFlags: u8 {
        const SLEEP_G       = 1 << 6;
        const INT2_ON_INT1  = 1 << 5;
        const DRDY_MASK     = 1 << 3;
        const I2C_DISABLE   = 1 << 2;
        const LPF1_SEL_G    = 1 << 1;
    }
}

bitflags::bitflags! {
    pub struct StatusFlags: u8 {
        const TDA   = 1 << 2;
        const GDA   = 1 << 1;
        const XLDA  = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct StatusMasterMainpageFlags: u8 {
        const WRT_ONCE_DONE     = 1 << 7;
        const SLAVE3_NACK       = 1 << 6;
        const SLAVE2_NACK       = 1 << 5;
        const SLAVE1_NACK       = 1 << 4;
        const SLAVE0_NACK       = 1 << 3;
        const SENS_HUB_ENDOP    = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct I3CBusAvbFlags: u8 {
        const I3C_Bus_Avb_Sel50u    = 0 << 3;
        const I3C_Bus_Avb_Sel2u     = 1 << 3;
        const I3C_Bus_Avb_Sel1m     = 2 << 3;
        const I3C_Bus_Avb_Sel25m    = 3 << 3;
        const PD_DIS_INT1           = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct TapCfg0Flags: u8 {
        const INT_CLR_ON_READ       = 1 << 6;
        const SLEEP_STATUS_ON_INT   = 1 << 5;
        const SLOPE_FDS             = 1 << 4;
        const TAP_X_EN              = 1 << 3;
        const TAP_Y_EN              = 1 << 2;
        const TAP_Z_EN              = 1 << 1;
        const LIR                   = 1 << 0;
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TempOdrRate {
    Disabled = 0,
    Hz1_6 = 1,
    Hz12_5 = 2,
    Hz52 = 3,
}

registers! {
    SensorHubReg, SENSORHUB_REG {
        FuncCfgAccess       = 0x01,
        // Slave 0
        Slv0Addr            = 0x15,
        Slv0SubAddr         = 0x16,
        Slv0Config          = 0x17,

        // Slave 1
        Slv1Addr            = 0x18,
        Slv1SubAddr         = 0x19,
        Slv1Config          = 0x1A,

        // Slave 2
        Slv2Addr            = 0x1B,
        Slv2SubAddr         = 0x1C,
        Slv2Config          = 0x1D,

        // Slave 3
        Slv3Addr            = 0x1E,
        Slv3SubAddr         = 0x1F,
        Slv3Config          = 0x20,

        DataWriteSrcMode    = 0x26,
        MasterConfig        = 0x14,

        SensorHub1Reg       = 0x02,
        SensorHub2Reg       = 0x03,
        SensorHub3Reg       = 0x04,
        SensorHub4Reg       = 0x05,
        SensorHub5Reg       = 0x06,
        SensorHub6Reg       = 0x07,
        SensorHub7Reg       = 0x08,
        SensorHub8Reg       = 0x09,
        SensorHub9Reg       = 0x0A,
        SensorHub10Reg      = 0x0B,
        SensorHub11Reg      = 0x0C,
        SensorHub12Reg      = 0x0D,
        SensorHub13Reg      = 0x0E,
        SensorHub14Reg      = 0x0F,
        SensorHub15Reg      = 0x10,
        SensorHub16Reg      = 0x11,
        SensorHub17Reg      = 0x12,
        SensorHub18Reg      = 0x13,

        StatusMaster        = 0x22,
    }
}

bitflags::bitflags! {
    pub struct MasterConfigFlags: u8 {
        const RST_MASTER_REGS   = 1 << 7;
        const WRITE_ONCE        = 1 << 6;
        const START_CONFIG      = 1 << 5;
        const PASS_THROUGH_MODE = 1 << 4;
        const SHUB_PU_EN        = 1 << 3;
        const MASTER_ON         = 1 << 2;
    }
}

pub const MASTER_CONFIG_AUX_SENSOR_LOC: u8 = 0;
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MasterConfigAuxSensor {
    OneSensor   = 0,
    TwoSensor   = 1,
    ThreeSensor = 2,
    FourSensor  = 3,
}

bitflags::bitflags! {
    pub struct StatusMasterFlags: u8 {
        const WRT_ONCE_DONE     = 1 << 7;
        const SLAVE3_NACK       = 1 << 6;
        const SLAVE2_NACK       = 1 << 5;
        const SLAVE1_NACK       = 1 << 4;
        const SLAVE0_NACK       = 1 << 3;
        const SENS_HUB_ENDOP    = 1 << 0;
    }
}


pub const BATCH_EXT_SENS_0_EN_LOC: u8 = 3;
pub const SHUB_ODR_LOC: u8 = 6;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ShubOdrRate {
    Hz104   = 0,
    Hz52    = 1,
    Hz26    = 2,
    Hz12_5  = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegWakeup {
    PageSel            = 0x02,  // Selects embedded register page
    PageAddress        = 0x08,  // Used to address sub-registers
    EmbFuncEnA         = 0x04,  // Enable embedded functions
    EmbFuncEnB         = 0x05,
    EmbFuncInt1        = 0x0A,  // Route events to INT1
    TapCfg0            = 0x56,
    TapCfg1            = 0x57,
    WakeUpThresh       = 0x5B,
    WakeUpDur          = 0x5C,
    WakeUpSrc          = 0x1B,  // Note: back on main register page
}
impl_register!(RegWakeup);

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegSelfTest {
    Ctrl3C             = 0x12, // Self-test enable bits (ST_XL, ST_G)
    Ctrl5C             = 0x14, // Rounding / update modes
    Ctrl10C            = 0x19, // Enable accelerometer and gyro
    OutXlX             = 0x28, // XL output X (low)
    OutXlY             = 0x2A,
    OutXlZ             = 0x2C,
    OutGxL             = 0x22,
    OutGyL             = 0x24,
    OutGzL             = 0x26,
}
impl_register!(RegSelfTest);


// -- Bit offset constants --

pub const CTRL3_C_IF_INC_LOC: u8 = 2;
pub const CTRL3_C_BDU_LOC: u8 = 6;

pub const CTRL1_XL_ODR_LOC: u8 = 4;
pub const CTRL1_XL_FS_LOC: u8 = 2;
pub const CTRL1_XL_LP_MODE_LOC: u8 = 0;

pub const CTRL2_G_ODR_LOC: u8 = 4;
pub const CTRL2_G_FS_LOC: u8 = 2;
pub const CTRL2_G_FS_125_LOC: u8 = 1;

pub const CTRL10_C_XEN_LOC: u8 = 0;
pub const CTRL10_C_YEN_LOC: u8 = 1;
pub const CTRL10_C_ZEN_LOC: u8 = 2;
pub const CTRL10_C_GYRO_EN_LOC: u8 = 4;

pub const FIFO_CTRL3_DEC_TS_LOC: u8 = 4;
pub const FIFO_CTRL3_DEC_GYRO_LOC: u8 = 2;
pub const FIFO_CTRL3_DEC_XL_LOC: u8 = 0;

pub const FIFO_CTRL4_ODR_LOC: u8 = 0;
pub const FIFO_CTRL4_MODE_LOC: u8 = 3;

pub const INT1_CTRL_FIFO_WTM_LOC: u8 = 1;

pub const FUNC_CFG_ACCESS_FUNC_CFG_EN_LOC: u8 = 7;
pub const FUNC_CFG_ACCESS_SENSOR_HUB_EN_LOC: u8 = 3;

pub const MASTER_CONFIG_SHUB_EN_LOC: u8 = 2;

pub const EMB_FUNC_EN_A_FSM_EN_LOC: u8 = 0;

pub const EMB_FUNC_EN_B_FSM_INIT_EN_LOC: u8 = 0;

// Self-test
pub const CTRL5_C_ST_XL_LOC: u8 = 0;
pub const CTRL5_C_ST_G_LOC: u8 = 2;

#[repr(u8)]
pub enum TphSelMode {
    Formula1 = 0,
    Formula2 = 1,
}

#[repr(u8)]
pub enum S4SRRMode {
    ResolutionMode0 = 0,
    ResolutionMode1 = 1,
    ResolutionMode2 = 2,
    ResolutionMode3 = 3,
}

#[repr(u8)]
pub enum Odr {
    PowerDown   = 0,
    Hz12_5      = 1,
    Hz26        = 2,
    Hz52        = 3,
    Hz104       = 4,
    Hz208       = 5,
    Hz416       = 6,
    Hz833       = 7,
    Hz1667      = 8,
    Hz3333      = 9,
    Hz6667      = 10,
}

pub const CTRL1_XL_ODR_MASK: u8 = 0b1111 << CTRL1_XL_ODR_LOC;
pub const CTRL1_XL_LP_MODE_MASK: u8 = 0b1 << CTRL1_XL_LP_MODE_LOC;
pub const CTRL2_G_ODR_MASK: u8 = 0b1111 << CTRL2_G_ODR_LOC;


#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AccelFullScale {
    G2  = 0b00,
    G4  = 0b10,
    G8  = 0b11,
    G16 = 0b01,
}
pub const CTRL1_XL_FS_MASK: u8 = 0b11 << CTRL1_XL_FS_LOC;

#[repr(u8)]
pub enum GyroFullScale {
    DPS250 = 0b000,
    DPS125 = 0b001,
    DPS500 = 0b010,
    DPS1000 = 0b100,
    DPS2000 = 0b110,
}
pub const CTRL2_G_FS_MASK: u8 = 0b111 << CTRL2_G_FS_LOC;

// FIFO Tags
registers! {
    FifoTags, FIFO_TAGS {
        GyroscopeNc         = 0x01,
        AccelerometerNc     = 0x02,
        Temperature         = 0x03,
        Timestamp           = 0x04,
        CfgChange           = 0x05,
        AccelerometerNcT2   = 0x06,
        AccelerometerNcT1   = 0x07,
        Accelerometer2xC    = 0x08,
        Accelerometer3xC    = 0x09,
        GyroscopeNcT2       = 0x0A,
        GyroscopeNcT1       = 0x0B,
        Gyroscope2xC        = 0x0C,
        Gyroscope3xC        = 0x0D,
        SensorHubSlave0     = 0x0E,
        SensorHubSlave1     = 0x0F,
        SensorHubSlave2     = 0x10,
        SensorHubSlave3     = 0x11,
        StepCounter         = 0x12,
        SensorHubNack       = 0x19,
    }
}

impl TryFrom<u8> for FifoTags {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(FifoTags::GyroscopeNc),
            0x02 => Ok(FifoTags::AccelerometerNc),
            0x03 => Ok(FifoTags::Temperature),
            0x04 => Ok(FifoTags::Timestamp),
            0x05 => Ok(FifoTags::CfgChange),
            0x06 => Ok(FifoTags::AccelerometerNcT2),
            0x07 => Ok(FifoTags::AccelerometerNcT1),
            0x08 => Ok(FifoTags::Accelerometer2xC),
            0x09 => Ok(FifoTags::Accelerometer3xC),
            0x0A => Ok(FifoTags::GyroscopeNcT2),
            0x0B => Ok(FifoTags::GyroscopeNcT1),
            0x0C => Ok(FifoTags::Gyroscope2xC),
            0x0D => Ok(FifoTags::Gyroscope3xC),
            0x0E => Ok(FifoTags::SensorHubSlave0),
            0x0F => Ok(FifoTags::SensorHubSlave1),
            0x10 => Ok(FifoTags::SensorHubSlave2),
            0x11 => Ok(FifoTags::SensorHubSlave3),
            0x12 => Ok(FifoTags::StepCounter),
            0x19 => Ok(FifoTags::SensorHubNack),
            _ => Err(()),
        }
    }
}
impl core::fmt::UpperHex for FifoTags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // format the enum as its underlying u8 value
        write!(f, "{:#02X}", *self as u8)
    }
}
impl core::fmt::LowerHex for FifoTags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // format the enum as its underlying u8 value
        write!(f, "{:#02x}", *self as u8)
    }
}

// -- FIFO Temperature ODR values --
pub const FIFO_TEMP_ODR_LOC: u8 = 4;
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoOdrT {
    Disabled    = 0,
    Hz1_6       = 1,
    Hz12_5      = 2,
    Hz52        = 3,
}

// -- FIFO ODR values --
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoOdr {
    Disabled = 0,
    Hz12_5 = 1,
    Hz26 = 2,
    Hz52 = 3,
    Hz104 = 4,
    Hz208 = 5,
    Hz416 = 6,
    Hz833 = 7,
    Hz1660 = 8,
    Hz3330 = 9,
    Hz6660 = 10,
}

// -- FIFO Modes --
pub const FIFO_MODE_LOC: u8 = 0;
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoMode {
    Bypass = 0,
    Fifo = 1,
    ContinuousToFifo = 3,
    BypassToContinuous = 4,
    Continuous = 6,
}

// -- Decimation settings --
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Decimation {
    Off = 0b000,
    EverySample = 0b001,
    Every2nd = 0b010,
    Every4th = 0b011,
    Every8th = 0b100,
    Every16th = 0b101,
    Every32nd = 0b110,
}

bitflags::bitflags! {
    pub struct Int1CtrlBitflags: u8 {
        const Int1DrdyXl    = 1 << 0;
        const Int1DrdyG     = 1 << 1;
        const Int1Boot      = 1 << 2;
        const Int1FifoTh    = 1 << 3;
        const Int1FifoOvr   = 1 << 4;
        const Int1FifoFull  = 1 << 5;
        const Int1CntBdr    = 1 << 6;
        const Int1DenRdy    = 1 << 7;
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoCtrl2Bitflags {
    WTM8                        = 1 << 0,
    NonCompressDataNotForced    = 0 << 1,
    NonCompressDataForced8      = 1 << 1,
    NonCompressDataForced16     = 2 << 1,
    NonCompressDataForced32     = 3 << 1,
    OdrChangeEnable             = 1 << 4,
    FifoCompressionEnable       = 1 << 6,
    StopOnWtm                   = 1 << 7,
}

pub const FIFO_BATCH_ODR_XL_LOC: u8 = 0;
pub const FIFO_BATCH_ODR_G_LOC: u8 = 4;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoBatchOdr {
    NotBatched  = 0,
    Hz12_5      = 1,
    Hz26        = 2,
    Hz52        = 3,
    Hz104       = 4,
    Hz208       = 5,
    Hz417       = 6,
    Hz833       = 7,
    Hz1667      = 8,
    Hz3333      = 9,
    Hz6667      = 10,
}

// -- Sensor hub register setup --
pub const SENSOR_HUB1_REG: u8 = 0x02; // First register for SH slave data output
pub const SHUB_OUT_LEN: usize = 18;   // Up to 18 bytes from sensor hub

bitflags::bitflags! {
    pub struct Ctrl3CFlags: u8 {
        const BOOT     = 1 << 7;
        const BDU      = 1 << 6;
        const H_LACTIVE = 1 << 5;
        const PP_OD    = 1 << 4;
        const SIM      = 1 << 3;
        const IF_INC   = 1 << 2;
        const BLE      = 1 << 1;
        const SW_RESET = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct Ctrl7GFlags: u8 {
        const G_HM_MODE         = 1 << 7;
        const HP_EN_G           = 1 << 6;
        const HPM_G16mHz        = 0 << 4;
        const HPM_G65mHz        = 1 << 4;
        const HPM_G260mHz       = 2 << 4;
        const HPM_G1_4Hz        = 3 << 4;
        const OIS_ON_EN         = 1 << 2;
        const USR_OFF_ON_OUT    = 1 << 1;
        const OIS_ON            = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct Ctrl9XlFlags: u8 {
        const DEN_X         = 1 << 7;
        const DEN_Y         = 1 << 6;
        const DEN_Z         = 1 << 5;
        const DEN_XL_G      = 1 << 4;
        const DEN_XL_EN     = 1 << 3;
        const DEN_LH        = 1 << 2;
        const I3C_DISABLE   = 1 << 1;
    }
}

#[repr(u8)]
pub enum FifoCtrl1 {
    FifoThresholdL = 0x07,
}

#[repr(u8)]
pub enum FifoCtrl2 {
    FifoThresholdH = 0x08,
}

#[repr(u8)]
pub enum FifoCtrl3Decimation {
    NoDecimation = 0b000,
    Every2nd     = 0b001,
    Every4th     = 0b010,
    Every8th     = 0b011,
    Every16th    = 0b100,
    Every32nd    = 0b101,
    Every64th    = 0b110,
    Every128th   = 0b111,
}

#[repr(u8)]
pub enum ShubReg {
    FuncCfgAccess = 0x01,
    Slv0DataOut0  = 0x02, // through 0x06 for external sensor data
    MasterConfig  = 0x14,
    Slv0Add       = 0x15,
    Slv0SubAdd    = 0x16,
    Slv0Config    = 0x17,
}

/// Alternate name for register 0x17 when used for data source configuration
pub const SHUB_DATA_WRITE_SRC_MODE_SUB: u8 = ShubReg::Slv0Config as u8;

/// Alternate use of 0x02 when accessing embedded PageSelect
pub const SHUB_PAGE_SELECT: u8 = ShubReg::Slv0DataOut0 as u8;


pub const FUNC_CFG_ACCESS_SHUB_EN_LOC: u8 = 3;
pub const MASTER_CONFIG_MASTER_ON_LOC: u8 = 0;
pub const SLV0_CONFIG_NUM_OPS_LOC: u8 = 3;

#[repr(u8)]
pub enum FuncCfgAccessMode {
    User        = 0,
    EmbFuncCfg  = 1,
    SensorHub   = 2,
    OisCtrl     = 3,
}
impl TryFrom<u8> for FuncCfgAccessMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(FuncCfgAccessMode::User),
            0x01 => Ok(FuncCfgAccessMode::EmbFuncCfg),
            0x02 => Ok(FuncCfgAccessMode::SensorHub),
            0x03 => Ok(FuncCfgAccessMode::OisCtrl),
           _ => Err(()),
        }
    }
}
bitflags::bitflags! {
    pub struct FuncCfgAccessFlags: u8 {
        const EMBFUNC_CFG_ACCESS    = 1 << 7;
        const SHUB_REG_ACCESS       = 1 << 6;
        const OIS_CTRL_ACCESS       = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct PinCtrlFlags: u8 {
        const OIS_PU_DIS    = 1 << 7;
        const SDO_PU_EN     = 1 << 6;
    }
}

bitflags::bitflags! {
    pub struct PinC: u8 {
        const SHUB_PU_EN     = 0b00010000;
        const AUX_SENS_ON    = 0b00000100;
        const MASTER_ON      = 0b00000001;
    }
}

#[repr(u8)]
pub enum SelfTestMode {
    Disabled      = 0b00,
    PositiveSign  = 0b01,
    NegativeSign  = 0b11,
}

pub const CTRL5C_ST_XL_LOC: u8 = 0; // 1:0
pub const CTRL5C_ST_G_LOC: u8 = 2;  // 3:2

bitflags::bitflags! {
    pub struct UiIntOisBitFlags: u8 {
        const SPI2_READ_EN  = 1 << 3;
        const DEN_LH_OIS    = 1 << 5;
        const LVL2_OIS      = 1 << 6;
        const INT2_DRDY_OIS = 1 << 7;
    }
}

registers! (
    EmbeddedFuncReg, EMBEDDED_FUNC_REG {
        FuncCfgAccess       = 0x01,
        PageSel             = 0x02,
        EmbFuncEnA          = 0x04,
        EmbFuncEnB          = 0x05,
        PageAddress         = 0x08,
        PageValue           = 0x09,
        EmbFuncInt1         = 0x0A,
        FsmInt1A            = 0x0B,
        FsmInt1B            = 0x0C,
        MlcInt1             = 0x0D,
        EmbFuncInt2         = 0x0E,
        FsmInt2A            = 0x0F,
        FsmInt2B            = 0x10,
        MlcInt2             = 0x11,
        EmbFuncStatus       = 0x12,
        FsmStatusA          = 0x13,
        FsmStatusB          = 0x14,
        MlcStatus           = 0x15,
        PageRw              = 0x17,
        EmbFuncFifoCfg      = 0x44,
        FsmEnableA          = 0x46,
        FsmEnableB          = 0x47,
        FsmLongCounterL     = 0x48,
        FsmLongCounterH     = 0x49,
        FsmLongCounterClear = 0x4A,
        FsmOuts1            = 0x4C,
        FsmOuts2            = 0x4D,
        FsmOuts3            = 0x4E,
        FsmOuts4            = 0x4F,
        FsmOuts5            = 0x50,
        FsmOuts6            = 0x51,
        FsmOuts7            = 0x52,
        FsmOuts8            = 0x53,
        FsmOuts9            = 0x54,
        FsmOuts10           = 0x55,
        FsmOuts11           = 0x56,
        FsmOuts12           = 0x57,
        FsmOuts13           = 0x58,
        FsmOuts14           = 0x59,
        FsmOuts15           = 0x5A,
        FsmOuts16           = 0x5B,
        EmbFuncOdrCfgB      = 0x5F,
        EmbFuncOdrCfgC      = 0x60,
        StepCounterL        = 0x62,
        StepCounterH        = 0x63,
        EmbFuncSrc          = 0x64,
        EmbFuncInitA        = 0x66,
        EmbFuncInitB        = 0x67,
        Mlc0Src             = 0x70,
        Mlc1Src             = 0x71,
        Mlc2Src             = 0x72,
        Mlc3Src             = 0x73,
        Mlc4Src             = 0x74,
        Mlc5Src             = 0x75,
        Mlc6Src             = 0x76,
        Mlc7Src             = 0x77,
    }
);

bitflags::bitflags! {
    pub struct PageRwFlags: u8 {
        const EMB_FUNC_LIR  = 1 << 7;
        const PAGE_WRITE    = 1 << 6;
        const PAGE_READ     = 1 << 5;
    }
}

pub const FSM_ODR_CFG_B_LOC: u8 = 3;
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FsmOdrCfgB {
    Hz12_5 = 0,
    Hz26 = 1,
    Hz52 = 2,
    Hz104 = 3,
}