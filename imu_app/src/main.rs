#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]

mod shared_i2c;
mod sensor_hub_control;
mod core_filters;
mod core_calibrate;
mod core_data_types;

use panic_rtt_target as _;
use cortex_m_rt::entry;

// use log::{info, warn, error, debug, trace};

// use lsm6dsox::Lsm6dsox;
// use lis3mdl::Lis3mdl;

use stm32f4xx_hal::{
    gpio::{gpiob::PB8, gpiob::PB9, Alternate, AF4, Edge, ExtiPin},
    i2c::I2c,
    pac::I2C1,
    rcc::Clocks,
    pac::GPIOB,
    time::Hertz,
};

type I2C = I2c<I2C1>;

use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::prelude::*;

use rtic::app;
use heapless::spsc::{Consumer, Producer, Queue};
use lsm6dsox::types::FifoSample;

use lsm6dsox::IMUStreamDriver;
use lsm6dsox::registers::*;
use lsm6dsox::configs::*;

use lis3mdl::registers::*;
use lis3mdl::configs::*;

use core_filters::*;

use rtt_target::{rprintln, rtt_init_print};

#[rtic::app(
device = stm32f4xx_hal::pac,
dispatchers = [EXTI0])]

mod app {
    use cortex_m_rt::entry;
    use rtt_target::{rprintln, rtt_init_print};
    // use log::LevelFilter;
    // use rtt_logger::RTTLogger;
    use lsm6dsox::{registers, IMUStreamDriver, Lsm6dsox};
    use lsm6dsox::registers::*;
    use lis3mdl::{registers as lis3mdl_registers, Lis3mdl, CONFIG_WAKEUP_LIS3MDL};
    use lis3mdl::registers:: *;
    use stm32f4xx_hal::{
        pac::{TIM2, Peripherals, GPIOB},               // Import TIM2 type from PAC
        prelude::*,                             // Import traits (TimerExt, etc.)
        timer::{Delay, Timer, DelayUs, Timer2},
        dwt::MonoTimer,
        rcc::Clocks,
        gpio::{Edge, GpioExt, PB5},
        time::Hertz,
    };
    use stm32f4xx_hal::i2c::Error as I2cError;
    use crate::{core_filters, I2C1};
    use heapless::spsc::{Consumer, Producer, Queue};
    use cortex_m::peripheral::scb::SystemHandler::SysTick;
    use rtic_monotonics::Monotonic;
    use systick_monotonic::*;
    // use log::{info, warn, error, debug, trace};

    use lsm6dsox::FifoSample;
    use stm32f4xx_hal::i2c::I2c;
    use embedded_hal::i2c::I2c as I2cTrait;
    use embedded_hal::delay::DelayNs;
    use rtic::Mutex;
    use crate::core_data_types::ThreeAxes;
    use crate::core_filters::{low_pass_filter_3axes, low_pass_filter_float, FilteredAxes, FilteredTemp};
    use crate::shared_i2c::SharedI2c;
    use crate::sensor_hub_control::SensorHub;
    use crate::core_calibrate::CalibratedGyro;
    use libm::*;

    const LSM6DSOX_ADDR: u8 = 0x6A;
    const LIS3MDL_ADDR: u8 = 0x1C;

    type HalI2c = stm32f4xx_hal::i2c::I2c<I2C1>;

    pub struct AccelState {
        initialized: bool,
        stable_count: u32,
        is_stable: bool,
        norm_last:f64,
    }

    pub const GYRO_DRIFT_LIMIT:f32 = 1.0;
    pub const X_SLOPE:f32 = 0.480922085;
    pub const Y_SLOPE:f32 = 0.089927936;
    pub const Z_SLOPE:f32 = 0.132195216;
    pub const X_INTERCEPT:f32 = -13.89444945;
    pub const Y_INTERCEPT:f32 = -2.59064349;
    pub const Z_INTERCEPT:f32 = -3.784211875;

    pub const LPF_ALPHA:f32 = 0.005;

#[shared]
    pub struct Shared {
        sample_producer: Producer<'static, FifoSample, 768>,
        sample_consumer: Consumer<'static, FifoSample, 768>,
        imu_stream: Lsm6dsox<SharedI2c, I2cError>,
        mag_stream: Lis3mdl<SharedI2c, I2cError>,
        delay: DelayUs<TIM2>, // Expected 84 MHz
        filtered_acc: FilteredAxes,
        filtered_gyr: FilteredAxes,
        filtered_mag: FilteredAxes,
        filtered_temp: FilteredTemp,
        gyro_cal: CalibratedGyro,
        acc_state: AccelState,
    }

    #[local]
    struct Local {
        mono: Systick<1000>,
        int1_pin: PB5,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let _rtt_channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 4096,
                    mode: rtt_target::ChannelMode::NoBlockSkip
                }
            }
            down: {
                0: {
                    size: 16
                }
            }
        };
        rtt_target::set_print_channel(_rtt_channels.up.0);
        // rtt_init_print!();
        rprintln!("RTIC #[init] started");

        let gyro_cal = CalibratedGyro::new(0.5, 100);
        let mut acc_state = AccelState {
            initialized:false,
            stable_count:0,
            is_stable:false,
            norm_last:0.0,
        };

        // info!("Startup complete");
        rprintln!("RTT init test print");

        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr
            .sysclk(84.MHz())  // Request 84 MHz SYSCLK via PLL
            .freeze();

        // Create a delay abstraction based on timer TIM2
        let mut delay: DelayUs<TIM2> = dp.TIM2.delay_us(&clocks);

        rprintln!("SYSCLK: {} Hz", clocks.sysclk());
        rprintln!("PCLK1:  {} Hz", clocks.pclk1());
        rprintln!("PCLK2:  {} Hz", clocks.pclk2());


        // Initialize SYSCFG
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;

        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb8.into_alternate::<4>().set_open_drain();
        let sda = gpiob.pb9.into_alternate::<4>().set_open_drain();
        let mut int1_pin = gpiob.pb5.into_floating_input();
        let mut int2_pin = gpiob.pb4.into_floating_input();

        int1_pin.make_interrupt_source(&mut syscfg);
        int1_pin.enable_interrupt(&mut exti);
        int1_pin.trigger_on_edge(&mut exti, Edge::Rising);

        let mono = Systick::new(ctx.core.SYST, 84_000_000);

        let mut i2c = HalI2c::new(dp.I2C1, (scl, sda), 100_u32.kHz(), &clocks);
        SharedI2c::init(i2c);

        let mut imu: Lsm6dsox<SharedI2c, I2cError> = Lsm6dsox::new(SharedI2c, LSM6DSOX_ADDR);
        let mut mag: Lis3mdl<SharedI2c, I2cError> = Lis3mdl::new(SharedI2c, LIS3MDL_ADDR);

        // imu.init().unwrap();
        // mag.init().unwrap();

        rprintln!("I2C init complete");

        scan_i2c_bus(&mut imu.i2c());

        rprintln!("Reading WHO_AM_I registers...");

        match read_who_am_i(&mut imu.i2c(), LSM6DSOX_ADDR, 0x0F) {
            Ok(who) => rprintln!("LSM6DSOX WHO_AM_I = 0x{:02X}", who),
            Err(_) => rprintln!("Failed to read LSM6DSOX WHO_AM_I"),
        }

        match read_who_am_i(&mut imu.i2c(), LIS3MDL_ADDR, 0x0F) {
            Ok(who) => rprintln!("LIS3MDL WHO_AM_I = 0x{:02X}", who),
            Err(_) => rprintln!("Failed to read LIS3MDL WHO_AM_I"),
        }

        let mut filtered_acc = FilteredAxes {
            data: ThreeAxes { x: 0.0, y: 0.0, z: 0.0 },
            initialized: false,
        };
        let mut filtered_gyr = FilteredAxes {
            data: ThreeAxes { x: 0.0, y: 0.0, z: 0.0 },
            initialized: false,
        };
        let mut filtered_mag = FilteredAxes {
            data: ThreeAxes { x: 0.0, y: 0.0, z: 0.0 },
            initialized: false,
        };
        let mut filtered_temp = FilteredTemp {
            data: 0.0,
            initialized: false,
        };

        // Initialize queue
        let (prod, cons) = cortex_m::singleton!(: Queue<FifoSample, 768> = Queue::new()).unwrap().split();


        // Disable embedded functions
        // imu.write_reg(MainReg::FuncCfgAccess.into(), 0x80).unwrap(); // Enable access to embedded registers
        // // Clear sensor hub slave configs
        // for reg in 0x15..=0x22 {
        //     imu.write_reg(reg, 0x00).unwrap();
        // }
        // imu.write_reg(MainReg::FuncCfgAccess.into(), 0x00).unwrap(); // Exit embedded register access

        rprintln!("Executing LSM6DSOX SW_RESET");
        imu.write_reg(lsm6dsox::registers::MainReg::Ctrl3C.into(), Ctrl3CFlags::SW_RESET.bits()).unwrap();
        loop {
            if imu.read_reg(lsm6dsox::registers::MainReg::Ctrl3C.into()).unwrap() & Ctrl3CFlags::SW_RESET.bits() == 0 {
                break;
            }
            delay.delay_ms(100_u32);
        }

        rprintln!("Dumping MAIN_REGS config");
        imu.dump_config(MAIN_REGS).unwrap();
        rprintln!("Dumping EMBEDDED_FUNC_REG config");
        imu.select_register_bank(FuncCfgAccessMode::EmbFuncCfg).unwrap();
        imu.dump_config(EMBEDDED_FUNC_REG).unwrap();
        imu.select_register_bank(FuncCfgAccessMode::User).unwrap();
        rprintln!("Dumping SENSORHUB_REG config");
        imu.select_register_bank(FuncCfgAccessMode::SensorHub).unwrap();
        imu.dump_config(SENSORHUB_REG).unwrap();
        imu.select_register_bank(FuncCfgAccessMode::User).unwrap();
        rprintln!("Dumping MAG_REGS config");
        mag.dump_config(MAG_REGS).unwrap();
        rprintln!("Dumping debug config complete");

        rprintln!("Initializing SensorHub Config");
        let mut hub = SensorHub::new(&mut imu, &mut mag, &mut delay);
        hub.configure().unwrap();

        rprintln!("9DOF config complete");

        (
            Shared {
                sample_producer: prod,
                sample_consumer: cons,
                imu_stream: imu,
                mag_stream: mag,
                delay,
                filtered_acc,
                filtered_gyr,
                filtered_mag,
                filtered_temp,
                gyro_cal,
                acc_state,
            },
            Local {
                mono,
                int1_pin,
            }
        )
    }

    #[task(binds = EXTI9_5, priority = 2, shared = [imu_stream, sample_producer], local = [int1_pin])]
    fn on_fifo_interrupt(mut cx: on_fifo_interrupt::Context) {
        cx.local.int1_pin.clear_interrupt_pending_bit();

        cx.shared.imu_stream.lock(|imu| {
            cx.shared.sample_producer.lock(|queue| {
                let count = imu.fifo_samples_available().unwrap_or(0);
                rprintln!("on_fifo_interrupt count: {}", count);
                // Get gyro settling status
                let gyro_settling:u8 = imu.get_gyro_settling();

                for _ in 0..count {
                    if let Ok(sample) = imu.read_fifo_sample() {
                        // if gyro_settling == 0 {
                            if queue.enqueue(sample).is_err() {
                                rprintln!("on_fifo_interrupt enqueue error");
                            }
                        // } else {
                        //     rprintln!("on_fifo_interrupt gyro settling");
                        // }
                    }
                }
            });
        });
    }

    #[task(priority = 1, shared = [imu_stream, sample_consumer, filtered_acc, filtered_gyr, filtered_mag, filtered_temp, gyro_cal, acc_state], local = [mono])]
    async fn process_samples(mut cx: process_samples::Context) {
        let mut las: f32 = 0.0;
        let mut ars: f32 = 0.0;
        let mut current = ThreeAxes { x: 0.0, y: 0.0, z: 0.0 };
        let mut current_adjusted_temp:f32 = 0.0;


        rprintln!("Processing samples...");
        
        // Get linear acceleration sensitivity
        cx.shared.imu_stream.lock(|imu| {
            match imu.get_linear_acc_sensitivty() {
                Ok(Some(sensitivity)) => las = sensitivity,
                Ok(None) => {
                    rprintln!("get_linear_acc_sensitivty not available (sensor off)");
                }
                Err(e) => {
                    rprintln!("Failed to read get_linear_acc_sensitivty: {:?}", e);
                }
            }
            match imu.get_angular_rate_sensitivty() {
                Ok(Some(sensitivity)) => ars = sensitivity,
                Ok(None) => {
                    rprintln!("get_angular_rate_sensitivty not available (sensor off)");
                }
                Err(e) => {
                    rprintln!("Failed to read get_angular_rate_sensitivty: {:?}", e);
                }
            }
        });

        let mut cal_gyro = ThreeAxes { x: 0.0, y: 0.0, z: 0.0 };

        // Process samples using lock mechanism
        cx.shared.sample_consumer.lock(|consumer| {
            while let Some(sample) = consumer.dequeue() {
                match sample {
                    FifoSample::Accel(accel_data) => {
                        let x: f32 = accel_data[0] as f32 * las;
                        let y: f32 = accel_data[1] as f32 * las;
                        let z: f32 = accel_data[2] as f32 * las;
                        let mut current_accel = ThreeAxes { x, y, z };
                        cx.shared.filtered_acc.lock(|mut filtered_acc| {
                            low_pass_filter_3axes(&current_accel, &mut filtered_acc, LPF_ALPHA);
                            rprintln!("Accel, {:.3}, {:.3}, {:.3}", filtered_acc.data.x, filtered_acc.data.y, filtered_acc.data.z);
                            cx.shared.acc_state.lock(|mut acc_state| {
                                if !acc_state.initialized {
                                    acc_state.norm_last = calculate_sqrt(
                                        (filtered_acc.data.x*filtered_acc.data.x +
                                            filtered_acc.data.y*filtered_acc.data.y +
                                            filtered_acc.data.z*filtered_acc.data.z) as f64).unwrap();
                                    acc_state.initialized = true;
                                    rprintln!("Accel initialized");
                                } else {
                                    let norm = calculate_sqrt((filtered_acc.data.x*filtered_acc.data.x +
                                                                        filtered_acc.data.y*filtered_acc.data.y +
                                                                        filtered_acc.data.z*filtered_acc.data.z) as f64).unwrap();
                                    let norm_diff = (norm - acc_state.norm_last).abs();
                                    rprintln!("Accel change, {:.3}, {:.3}, {:.3}", norm_diff, norm, acc_state.norm_last);
                                    if norm_diff > 2.0 {
                                        // rprintln!("Accel change, {:.3}, {:.3}, {:.3}", filtered_acc.data.x, filtered_acc.data.y, filtered_acc.data.z);
                                        acc_state.is_stable = false;
                                        acc_state.stable_count = 0;
                                        acc_state.norm_last = norm;
                                    } else {
                                        if acc_state.stable_count < 52 { // ODR = 1 sec
                                            acc_state.stable_count += 1;
                                        } else {
                                            acc_state.is_stable = true;
                                        }

                                        rprintln!("acc_state.is_stable {}", acc_state.is_stable);
                                    }
                                }
                                cx.shared.gyro_cal.lock(|mut gyro_cal| {
                                    if acc_state.is_stable && gyro_cal.is_calibrated {
                                        if cal_gyro.x.abs() > GYRO_DRIFT_LIMIT ||
                                            cal_gyro.y.abs() > GYRO_DRIFT_LIMIT ||
                                            cal_gyro.z.abs() > GYRO_DRIFT_LIMIT
                                        {
                                            // gyro_cal.reset_calibration();
                                        }
                                    }
                                });
                            });
                        });
                    }
                    FifoSample::Gyro(gyro_data) => {
                        cx.shared.filtered_temp.lock(|filtered_temp| {
                            // let x: f32 = temp_intercept_correction(gyro_data[0] as f32 * ars, current_adjusted_temp, X_SLOPE, X_INTERCEPT);
                            // let y: f32 = temp_intercept_correction(gyro_data[1] as f32 * ars, current_adjusted_temp, Y_SLOPE, Y_INTERCEPT);
                            // let z: f32 = temp_intercept_correction(gyro_data[2] as f32 * ars, current_adjusted_temp, Z_SLOPE, Z_INTERCEPT);
                            // let x: f32 = temp_slope_correction(gyro_data[0] as f32 * ars, current_adjusted_temp, X_SLOPE, 25.0);
                            // let y: f32 = temp_slope_correction(gyro_data[1] as f32 * ars, current_adjusted_temp, Y_SLOPE, 25.0);
                            // let z: f32 = temp_slope_correction(gyro_data[2] as f32 * ars, current_adjusted_temp, Z_SLOPE, 25.0);
                            let x: f32 = gyro_temp_correction(gyro_data[0] as f32, filtered_temp.data);
                            let y: f32 = gyro_temp_correction(gyro_data[1] as f32, filtered_temp.data);
                            let z: f32 = gyro_temp_correction(gyro_data[2] as f32, filtered_temp.data);
                            let mut current_gyro = ThreeAxes { x, y, z };
                            cx.shared.gyro_cal.lock(|mut gyro_cal| {
                                cx.shared.filtered_gyr.lock(|mut filtered_gyr| {
                                    low_pass_filter_3axes(&current_gyro, &mut filtered_gyr, LPF_ALPHA);
                                    if gyro_cal.is_calibrated {
                                        cal_gyro.x = filtered_gyr.data.x - gyro_cal.data.x;
                                        cal_gyro.y = filtered_gyr.data.y - gyro_cal.data.y;
                                        cal_gyro.z = filtered_gyr.data.z - gyro_cal.data.z;
                                        cx.shared.acc_state.lock(|mut acc_state| {
                                            if acc_state.is_stable {
                                                // rprintln!("Gyro offset, {:.3}, {:.3}, {:.3}", gyro_cal.data.x, gyro_cal.data.y, gyro_cal.data.z);
                                                rprintln!("Gyro calibrated, {:.3}, {:.3}, {:.3}", cal_gyro.x, cal_gyro.y, cal_gyro.z);
                                            }
                                        });
                                    } else {
                                        rprintln!("Gyro, {:.3}, {:.3}, {:.3}", filtered_gyr.data.x, filtered_gyr.data.y, filtered_gyr.data.z);
                                        gyro_cal.feed(filtered_gyr.data);
                                        rprintln!("Gyro Cal Count: {}", gyro_cal.get_cal_count());
                                    }
                                });
                            });
                        });
                    }
                    FifoSample::Mag(mag_data) => {
                        rprintln!("Mag: {:?}", mag_data);
                    }
                    FifoSample::Temperature(temp_data) => {
                        let temp_c = (temp_data[0] as f32) / 256.0 + 25.0;
                        current_adjusted_temp = temp_c; // start with raw; will overwrite with filtered below

                        cx.shared.filtered_temp.lock(|filtered_temp| {
                            if !filtered_temp.initialized {
                                filtered_temp.data = temp_c;
                                filtered_temp.initialized = true;
                            } else {
                                filtered_temp.data = low_pass_filter_float(temp_c, filtered_temp.data, LPF_ALPHA);
                            }
                            // propagate filtered temp to the rest of the loop (e.g., gyro temp compensation)
                            current_adjusted_temp = filtered_temp.data;
                        });

                        // rprintln!("Temperature: {:#.1} degC", temp_c);
                        cx.shared.acc_state.lock(|acc_state| {
                            if acc_state.is_stable {
                                // Print filtered temperature when stable
                                rprintln!("Temperature: {},", current_adjusted_temp);
                            }
                        });
                    }
                    FifoSample::Unknown(tag) => {
                        rprintln!("Unknown sample type: {:#02X}", tag);
                    }
                }
            }
        });
    }

    fn gyro_temp_correction(gyro_data: f32, temp: f32) -> f32 {
        // Reference temperature (°C) at which the gyro was calibrated
        const REF_TEMP_C: f32 = 0.0;
        // Temperature drift slope (units of gyro_data per °C).
        // Adjust this based on your sensor's characterization.
        const SLOPE: f32 = 0.7;

        let delta_t = temp - REF_TEMP_C;
        let adjustment = delta_t * SLOPE;

        // Apply correction: if your measured drift increases with temperature,
        // subtract the adjustment; if it decreases, flip the sign or SLOPE.
        gyro_data - adjustment
    }

    fn parse_fifo_frame(frame: &[u8; 7]) {
        let tag = frame[0];
        let x = i16::from_le_bytes([frame[1], frame[2]]);
        let y = i16::from_le_bytes([frame[3], frame[4]]);
        let z = i16::from_le_bytes([frame[5], frame[6]]);

        rprintln!("Tag: 0x{:02X}, X: {}, Y: {}, Z: {}", tag, x, y, z);
    }

    fn scan_i2c_bus<I2C, E>(i2c: &mut I2C)
    where
        I2C: I2cTrait<Error = E>,
        E: core::fmt::Debug,
    {
        rprintln!("Scanning I2C bus...");
        for addr in 0x08..=0x77 {
            let mut buf = [0u8];
            match i2c.write_read(addr, &[0x00], &mut buf) {
                Ok(_) => rprintln!(" - Found device at 0x{:02X}", addr),
                Err(_) => {} // No response or error, skip
            }
        }
    }

    fn read_who_am_i<I2C, E>(i2c: &mut I2C, addr: u8, reg: u8) -> Result<u8, E>
    where
        I2C: I2cTrait<Error = E>,
        E: core::fmt::Debug,
    {
        let mut buf = [0u8];
        i2c.write_read(addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    #[idle(shared = [sample_consumer, delay])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            let queue_len = ctx.shared.sample_consumer.lock(|consumer| consumer.len());
            // Defer processing to lower-priority task
            if queue_len > 0 {
                process_samples::spawn().ok();
                // cortex_m::asm::wfi(); // Wait for interrupt (power-efficient idle)
            } else {
                ctx.shared.delay.lock(|delay| delay.delay_ms(100_u32));
            }
        }
    }

    fn calculate_sqrt(x: f64) -> Option<f64> {
        // Check if the input x is non-negative
        if x >= 0.0 {
            // If x is non-negative, calculate its square root using sqrt() method and wrap the result in Some
            Some(sqrt(x))
        } else {
            // If x is negative, return None
            None
        }
    }

    pub fn temp_slope_correction(measured: f32, temp_c: f32, slope: f32, offset: f32) -> f32 {
        measured - (slope * (temp_c - offset))
    }

    pub fn temp_intercept_correction(measured: f32, temp_c: f32, slope: f32, intercept: f32) -> f32 {
        measured - (slope * temp_c + intercept)
    }

    // fn wait_for_shub_ready() -> Result<(), Error<E>> {
    //     loop {
    //         let status = imu.read_register(MainReg::StatusMasterMainpage)?;
    //         if status & 0x01 == 0 {
    //             break;
    //         }
    //     }
    //     Ok(())
    // }
}