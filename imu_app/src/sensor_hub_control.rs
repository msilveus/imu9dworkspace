
use stm32f4xx_hal::i2c::Error as I2cError;
use crate::shared_i2c::SharedI2c;
use stm32f4xx_hal::pac::TIM2;
use stm32f4xx_hal::timer::{DelayUs as TimerDelay};
use embedded_hal::delay::DelayNs;
use rtt_target::rprintln;
use lis3mdl::Lis3mdl;
use lis3mdl::registers::*;
use lis3mdl::configs::*;
use lsm6dsox::{FifoSample, Lsm6dsox, IMUStreamDriver};
use lsm6dsox::registers::*;
use lsm6dsox::configs::*;


pub struct SensorHub<'a> {
    imu: &'a mut Lsm6dsox<SharedI2c, I2cError>,
    mag: &'a mut Lis3mdl<SharedI2c, I2cError>,
    delay: &'a mut TimerDelay<TIM2>,
}

impl<'a> SensorHub<'a> {
    pub fn new(
        imu: &'a mut Lsm6dsox<SharedI2c, I2cError>,
        mag: &'a mut Lis3mdl<SharedI2c, I2cError>,
        delay: &'a mut TimerDelay<TIM2>,
    ) -> Self {
        Self { imu, mag, delay }
    }

    pub fn configure(&mut self) -> Result<(), I2cError> {
        let mut loop_counter:u32 = 0;

        self.imu.write_reg(MainReg::Ctrl9Xl as u8, Ctrl9XlFlags::I3C_DISABLE.bits()).unwrap();
        // Reset Mag config logic
        rprintln!("Reset Mag config logic");
        self.mag.write_reg(MagReg::CtrlReg2 as u8, CtrlReg2Bitflags::SoftReset as u8).unwrap();
        self.delay.delay_us(100);
        self.mag.write_reg(MagReg::CtrlReg2 as u8, 0).unwrap();
        // Done resetting Mag config
        rprintln!("Done resetting Mag config");
        // Reset Acc config logic
        rprintln!("Reset Acc config logic");
        self.imu.write_reg(MainReg::Ctrl3C as u8, Ctrl3CFlags::SW_RESET.bits()).unwrap();
        while self.imu.read_reg(MainReg::Ctrl3C as u8).unwrap() & Ctrl3CFlags::SW_RESET.bits() != 0 {
            self.delay.delay_us(100);
        }
        // Done resetting Acc config
        rprintln!("Done resetting Acc config");
        // Reset Master config logic
        rprintln!("Reset Master config logic");
        self.imu.write_reg(SensorHubReg::FuncCfgAccess as u8, FuncCfgAccessMode::SensorHub as u8).unwrap();
        self.imu.write_reg(SensorHubReg::MasterConfig as u8, MasterConfigFlags::RST_MASTER_REGS.bits()).unwrap();
        self.delay.delay_us(300);
        self.imu.write_reg(SensorHubReg::MasterConfig as u8, MasterConfigFlags::empty().bits()).unwrap();
        self.imu.write_reg(SensorHubReg::FuncCfgAccess as u8, FuncCfgAccessMode::User as u8).unwrap();
        // Done resetting Master config
        rprintln!("Done resetting Master config");
        // rprintln!("CONFIG_STREAMING");
        // self.imu.apply_config(CONFIG_STREAMING).unwrap();
        // rprintln!("CONFIG_INT_NOTIFICATION");
        // self.imu.apply_config(CONFIG_INT_NOTIFICATION).unwrap();
        // rprintln!("CONFIG_EMB_FUNCS");
        // self.imu.apply_config(CONFIG_EMB_FUNCS).unwrap();
        // rprintln!("CONFIG_SENSOR_HUB_LIS3MDL");
        // self.imu.apply_config(CONFIG_SENSOR_HUB_LIS3MDL).unwrap();
        // rprintln!("CONFIG_WAKEUP_LSM6DSOX");
        // self.imu.apply_config(CONFIG_WAKEUP_LSM6DSOX).unwrap();
        rprintln!("CONFIG_SENSOR_HUB_LIS3MDL_MIXED");
        self.imu.apply_any_config(CONFIG_SENSOR_HUB_LIS3MDL_MIXED).unwrap();
        rprintln!("CONFIG_WAKEUP_LIS3MDL");
        self.mag.apply_config(CONFIG_WAKEUP_LIS3MDL).unwrap();
        rprintln!("CONFIG_SH_MASTER_ON");
        self.imu.apply_config(CONFIG_SH_MASTER_ON).unwrap();
        self.delay.delay_ms(100);

        // loop {
        //     let mut buf = [0; 6];
        //     let mut status = self.mag.read_reg(MagReg::StatusReg as u8).unwrap();
        //     rprintln!("Mag status: {:#02X}", status);
        //     if (status & 0x07) == 0x07 {
        //         self.mag.read_bytes(MagReg::OutXL as u8, &mut buf).unwrap();
        //         rprintln!("mag data: {:?}", buf);
        //         break;
        //     }
        //     loop_counter += 1;
        //     if loop_counter > 10000 {
        //         return Err(I2cError::Timeout);
        //     }
        // }
        // loop {
        //     let mut status = self.imu.read_reg(lsm6dsox::registers::MainReg::StatusMasterMainpage as u8).unwrap();
        //     rprintln!("StatusMasterMainpage: {:#02X}", status);
        //     if (status & lsm6dsox::registers::StatusMasterFlags::SENS_HUB_ENDOP.bits()) == StatusMasterFlags::SENS_HUB_ENDOP.bits() {
        //         break;
        //     }
        //     loop_counter += 1;
        //     self.delay.delay_us(100);
        //     if loop_counter > 1000 { // 5 second timeout
        //         // return Err(I2cError::Timeout);
        //         break;
        //     }
        // }


        Ok(())
    }
}