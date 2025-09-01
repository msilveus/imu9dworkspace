#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]

extern crate alloc;

mod shared_i2c;
mod sensor_hub_control;
mod core_filters;
mod core_calibrate;
mod core_data_types;
mod core_uart_logger;
mod simple_ring_buffer;
mod core_temp_correction;

use alloc::format;
use alloc::string::ToString;
use panic_halt as _;
use cortex_m_rt::entry;
use core::mem::MaybeUninit;
use embedded_alloc::*;

use heapless::Vec;
use cortex_m::peripheral::NVIC;

// Bring fmt::Write into scope so `write!` works with heapless buffers
use core::fmt::Write;


use lsm6dsox::{
    registers::*,
    IMUStreamDriver,
    Lsm6dsox,
    types::FifoSample,
    configs::*,
};
use lis3mdl::{registers as lis3mdl_registers, Lis3mdl, CONFIG_WAKEUP_LIS3MDL};
use stm32f4xx_hal::{
    pac::{TIM2, Peripherals, GPIOB, USART1, Interrupt, I2C1},    // Import TIM2 type from PAC
    prelude::*,                                 // Import traits (TimerExt, etc.)
    timer::{Delay, Timer, DelayUs, Timer2},
    dwt::MonoTimer,
    rcc::Clocks,
    gpio::{Edge, GpioExt, PB5,PA5,PA9,PA10,Alternate,Output,PushPull, ExtiPin},
    time::Hertz,
    serial::{config::Config, Event as SerialEvent, Serial, Tx, Rx},
    i2c::I2c,
    i2c::Error as I2cError,
    interrupt::USART1 as USART1_IRQ,
};
// use crate::{core_filters, I2C1};
use heapless::{
    String as HString,
    spsc::{Consumer, Producer, Queue}
};
use core::cell::RefCell;
use critical_section::Mutex;

use log::{Log, Level, Metadata, Record, LevelFilter,trace, debug, info, warn, error, set_logger, set_max_level};

use lis3mdl::registers::*;
use lis3mdl::configs::*;

use core_filters::*;
use cortex_m::peripheral::scb::SystemHandler::SysTick;
use rtic_monotonics::{Monotonic as Monotonics};
use systick_monotonic::*;

use embedded_hal::i2c::I2c as I2cTrait;
use embedded_hal::delay::DelayNs;
// use rtic::Mutex;
use crate::core_data_types::ThreeAxes;
use crate::core_filters::{low_pass_filter_3axes, low_pass_filter_float, FilteredAxes, FilteredTemp};
use crate::shared_i2c::SharedI2c;
use crate::sensor_hub_control::SensorHub;
use crate::core_calibrate::CalibratedGyro;
use libm::*;
use stm32f4xx_hal::serial::config::StopBits;
use crate::app::parse_rx::LocalResources;
use alloc::boxed::Box;
use rtt_target::{rprintln, rtt_init_print};
use core_temp_correction::TempCompensator;
use embedded_hal::digital::InputPin;
use fugit::ExtU64;
use rtic_monotonic::Monotonic;
use rtic_monotonics::systick::prelude::*;

use rtic::app;

#[global_allocator]
static ALLOC: embedded_alloc::TlsfHeap = embedded_alloc::TlsfHeap::empty();

// A 16 KiB heap (adjust as needed)
#[link_section = ".uninit.heap"]
static mut HEAP_MEMORY: MaybeUninit<[u8; 16 * 1024]> = MaybeUninit::uninit();

type I2C = I2c<I2C1>;

// ---- Global Logger ----
const LOG_BUF_SIZE: usize   = 1024;
const TX_QSIZE: usize       = 4096;
const RX_QSIZE: usize       = 1024;
const LINE_BUF_SIZE: usize  = 128;

static mut TX_QUEUE: Queue<u8, TX_QSIZE> = Queue::new();

// Producer/Consumer handles (static references)
static mut TX_PROD: Option<Producer<'static, u8, TX_QSIZE>> = None;
static mut TX_CONS: Option<Consumer<'static, u8, TX_QSIZE>> = None;

static mut DEBUG_LOG_ACTIVE: bool = true;

type MyMono = Systick<1000>; // 1 kHz = 1 ms resolution
systick_monotonic!(Mono, 1000);

#[rtic::app(
device = stm32f4xx_hal::pac,
dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3])]

mod app {
    use super::*;

    const LSM6DSOX_ADDR: u8 = 0x6A;
    const LIS3MDL_ADDR: u8 = 0x1C;

    type HalI2c = stm32f4xx_hal::i2c::I2c<I2C1>;

    pub struct AccelState {
        initialized: bool,
        stable_count: u32,
        is_stable: bool,
        norm_last:f64,
    }

    pub struct LoggingStates {
        csvlog_initialized: bool,
        csvlog_header_printed: bool,
    }
    impl Default for LoggingStates {
        fn default() -> Self {
            Self {
                csvlog_initialized: false,
                csvlog_header_printed: false,
            }
        }
    }
    impl LoggingStates {
        fn set_csvlog_header_printed(&mut self, value:bool) -> &mut Self {
            self.csvlog_header_printed = value;
            self
        }
        fn set_csvlog_initialized(&mut self, value:bool) -> &mut Self {
            self.csvlog_initialized = value;
            self
        }
        fn is_csvlog_initialized(&self) -> bool {
            self.csvlog_initialized
        }
        fn is_csvlog_header_printed(&self) -> bool {
            self.csvlog_header_printed
        }
    }

    pub const GYRO_DRIFT_LIMIT:f32 = 1.0;
    pub const X_SLOPE:f32 = 0.480922085;
    pub const Y_SLOPE:f32 = 0.089927936;
    pub const Z_SLOPE:f32 = 0.132195216;
    pub const X_INTERCEPT:f32 = -13.89444945;
    pub const Y_INTERCEPT:f32 = -2.59064349;
    pub const Z_INTERCEPT:f32 = -3.784211875;

    pub const LPF_ALPHA:f32 = 0.0025;


    // Backing storage for SPSC queues (must be 'static)
    static mut TXQ: Queue<u8, TX_QSIZE> = Queue::new();
    static mut RXQ: Queue<u8, RX_QSIZE> = Queue::new();


    #[shared]
    pub struct Shared {
        int1_pin: PB5,
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
        tx: Tx<USART1>,
        // TX producer used by ISR to push into the transmit byte queue
        tx_prod: heapless::spsc::Producer<'static, u8, TX_QSIZE>,
        // TX consumer used by ISR to pull and transmit bytes
        tx_cons: heapless::spsc::Consumer<'static, u8, TX_QSIZE>,
        // RX consumer is shared so the parser task can drain bytes
        rx_cons: heapless::spsc::Consumer<'static, u8, RX_QSIZE>,
        // Onboard LED for demo commands
        led: PA5<Output<PushPull>>,
        logging_state: LoggingStates,
        tc: TempCompensator,
    }

    #[local]
    struct Local {
        // USART1 TX/RX halves used by the IRQ
        rx: Rx<USART1>,
        // RX producer used by ISR to push received bytes
        rx_prod: heapless::spsc::Producer<'static, u8, RX_QSIZE>,
        // Accumulate a line for command parsing
        line_buf: HString<LINE_BUF_SIZE>,
        log_line_buf: HString<LINE_BUF_SIZE>,
        last_active_int1_time: Option<rtic_monotonics::fugit::Instant<u32, 1, 1000>>,
    }

    pub struct UartLogger;

    static UART_LOGGER:UartLogger = UartLogger;

    impl Log for UartLogger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool { true }

        fn log(&self, record: &Record) { // enqueue log bytes into TX queue use core::fmt::Write;
            unsafe {
                if DEBUG_LOG_ACTIVE {
                    let mut msg: HString<128> = HString::new();
                    let level = record.level();
                    // let msg = format!("[{}] {}\r\n", level, record.args());
                    let _ = write!(&mut msg, "[{}] {}\r\n", level, record.args());
                    let _ = log_str::spawn(msg).ok();
                    // rprintln!("{}", msg);
                }
            }
        }
        fn flush(&self) {}
    }

    // static instance of the logger static UART_LOGGER: UartLogger = UartLogger;
    #[task(shared = [tx, tx_prod,tx_cons], priority = 8)]
    async fn log_str(mut ctx: log_str::Context, s: heapless::String<128>) {
        ctx.shared.tx_prod.lock(|tx_prod| {
            enqueue_tx_bytes(tx_prod, s.as_bytes());
            // enqueue_tx_bytes(tx_prod, b"\r\n");
        });
        cortex_m::peripheral::NVIC::pend(USART1_IRQ);
        ctx.shared.tx_cons.lock(|tx_cons| {
            ctx.shared.tx.lock(|tx| {
                let mut len = tx_cons.len();
                if len > 0 {
                    tx.listen();
                    // rprintln!("tx_cons: len = {}", len);
                }
            });
        });
    }
    /// Enqueue a byte slice to the TX queue (no blocking). Use from tasks via: /// cx.shared.tx_prod.lock(|tx| enqueue_tx_bytes(tx, ...));
    fn enqueue_tx_bytes( tx_prod: &mut heapless::spsc::Producer<'static, u8, TX_QSIZE>, bytes: &[u8], ) {
        for &b in bytes {
            if tx_prod.enqueue(b).is_err() {
                // Log or count failure
                // rprintln!("enqueue_tx_bytes TX queue full, dropping byte! {:#02X}", b);
            }
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        // Initialize queues
        let (prod, cons) = cortex_m::singleton!(: Queue<FifoSample, 768> = Queue::new()).unwrap().split();
        // Create UART byte queues (TX producer and RX consumer) expected by `Shared`
        // RX queue: producer is typically fed from ISR/driver, consumer is used by the app
        let (rx_prod, rx_cons) = cortex_m::singleton!(: Queue<u8, RX_QSIZE> = Queue::new()).unwrap().split();
        // TX queue: producer is used by the app, consumer is typically drained by ISR/driver
        let (tx_prod, tx_cons) = cortex_m::singleton!(: Queue<u8, TX_QSIZE> = Queue::new()).unwrap().split();

        let mut logging_state:LoggingStates = LoggingStates::default();


        set_logger(&UART_LOGGER)
            .map(|()| log::set_max_level(LevelFilter::Debug))
            .expect("Logger Init Failed");

        info!("RTIC #[init] started");

        let gyro_cal = CalibratedGyro::new(0.5, 100);
        let mut acc_state = AccelState {
            initialized:false,
            stable_count:0,
            is_stable:false,
            norm_last:0.0,
        };

        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr
            .sysclk(84.MHz())  // Request 84 MHz SYSCLK via PLL
            .pclk1(42.MHz()).
            freeze();

        // Create a delay abstraction based on timer TIM2
        let mut delay: DelayUs<TIM2> = dp.TIM2.delay_us(&clocks);

        debug!("SYSCLK: {} Hz", clocks.sysclk());
        debug!("PCLK1:  {} Hz", clocks.pclk1());
        debug!("PCLK2:  {} Hz", clocks.pclk2());

        let timer_hz = clocks.sysclk();

        Mono::start(ctx.core.SYST, timer_hz.to_Hz());

        // Initialize SYSCFG
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // PA5 as LED (LD2 on Nucleo-F401RE)
        let grn_led = gpioa.pa5.into_push_pull_output();

        let mut tc = TempCompensator::new(52.0); // e.g. ODR = 100 Hz

        // i2c pins
        let scl = gpiob.pb8.into_alternate::<4>().set_open_drain();
        let sda = gpiob.pb9.into_alternate::<4>().set_open_drain();
        // ext interrputs
        let mut int1_pin = gpiob.pb5.into_floating_input();
        let mut int2_pin = gpiob.pb4.into_floating_input();

        int1_pin.make_interrupt_source(&mut syscfg);
        int1_pin.enable_interrupt(&mut exti);
        int1_pin.trigger_on_edge(&mut exti, Edge::Rising);

        // USART1 pins: PA9 (TX), PA10 (RX), AF7
        let tx = gpioa.pa9.into_alternate::<7>();
        let rx = gpioa.pa10.into_alternate::<7>();

        // Keep Serial unified; Local expects `serial2` (not split halves)
        let serial2 = Serial::new(
            dp.USART1,
            (tx, rx),
            Config::default()
                .baudrate(921_600.bps())
                .wordlength_8()
                .parity_none()
                .stopbits(StopBits::STOP1),
            &clocks,
        ).unwrap();
        let (mut tx, mut rx) = serial2.split();
        rx.listen();
       info!("Hello from RTIC!\r\n");

        let mut last_active_int1_time: Option<rtic_monotonics::fugit::Instant<u32, 1, 1000>> = None;


        let mut i2c = HalI2c::new(dp.I2C1, (scl, sda), 100_u32.kHz(), &clocks);
        SharedI2c::init(i2c);

        let mut imu: Lsm6dsox<SharedI2c, I2cError> = Lsm6dsox::new(SharedI2c, LSM6DSOX_ADDR);
        let mut mag: Lis3mdl<SharedI2c, I2cError> = Lis3mdl::new(SharedI2c, LIS3MDL_ADDR);

        // imu.init().unwrap();
        // mag.init().unwrap();

        debug!("I2C init complete");

        scan_i2c_bus(&mut imu.i2c());

        debug!("Reading WHO_AM_I registers...");

        match read_who_am_i(&mut imu.i2c(), LSM6DSOX_ADDR, 0x0F) {
            Ok(who) => debug!("LSM6DSOX WHO_AM_I = 0x{:02X}", who),
            Err(_) => debug!("Failed to read LSM6DSOX WHO_AM_I"),
        }

        match read_who_am_i(&mut imu.i2c(), LIS3MDL_ADDR, 0x0F) {
            Ok(who) => debug!("LIS3MDL WHO_AM_I = 0x{:02X}", who),
            Err(_) => debug!("Failed to read LIS3MDL WHO_AM_I"),
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


        // Disable embedded functions
        // imu.write_reg(MainReg::FuncCfgAccess.into(), 0x80).unwrap(); // Enable access to embedded registers
        // // Clear sensor hub slave configs
        // for reg in 0x15..=0x22 {
        //     imu.write_reg(reg, 0x00).unwrap();
        // }
        // imu.write_reg(MainReg::FuncCfgAccess.into(), 0x00).unwrap(); // Exit embedded register access

        debug!("Executing LSM6DSOX SW_RESET");
        imu.write_reg(lsm6dsox::registers::MainReg::Ctrl3C.into(), Ctrl3CFlags::SW_RESET.bits()).unwrap();
        loop {
            if imu.read_reg(lsm6dsox::registers::MainReg::Ctrl3C.into()).unwrap() & Ctrl3CFlags::SW_RESET.bits() == 0 {
                break;
            }
            delay.delay_ms(100_u32);
        }

        debug!("Dumping MAIN_REGS config");
        imu.dump_config(MAIN_REGS).unwrap();
        debug!("Dumping EMBEDDED_FUNC_REG config");
        imu.select_register_bank(FuncCfgAccessMode::EmbFuncCfg).unwrap();
        imu.dump_config(EMBEDDED_FUNC_REG).unwrap();
        imu.select_register_bank(FuncCfgAccessMode::User).unwrap();
        debug!("Dumping SENSORHUB_REG config");
        imu.select_register_bank(FuncCfgAccessMode::SensorHub).unwrap();
        imu.dump_config(SENSORHUB_REG).unwrap();
        imu.select_register_bank(FuncCfgAccessMode::User).unwrap();
        debug!("Dumping MAG_REGS config");
        mag.dump_config(MAG_REGS).unwrap();
        debug!("Dumping debug config complete");

        debug!("Initializing SensorHub Config");
        let mut hub = SensorHub::new(&mut imu, &mut mag, &mut delay);
        hub.configure().unwrap();

        debug!("Init complete");

        (
            Shared {
                int1_pin,
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
                led: grn_led,
                rx_cons,     // keep for app-side dequeue of received bytes
                tx,
                tx_prod,
                tx_cons,
                logging_state,
                tc,
            },
            Local {
                rx,
                rx_prod,
                line_buf: HString::new(),
                log_line_buf: HString::new(),
                last_active_int1_time,
            }
        )
    }

    #[task(binds = EXTI9_5, priority = 1, shared = [int1_pin, imu_stream, sample_producer])]
    fn on_fifo_interrupt(mut cx: on_fifo_interrupt::Context) {
        cx.shared.int1_pin.lock(|pin| {
            pin.clear_interrupt_pending_bit();
        });

        cx.shared.imu_stream.lock(|imu| {
            cx.shared.sample_producer.lock(|queue| {
                let mut intflags_count = imu.fifo_samples_available().unwrap_or((0,0));
                let (mut intflags, mut count) = intflags_count;
                // debug!("on_fifo_interrupt count: {:#02x}:{}", intflags, count);
                // Get gyro settling status
                let gyro_settling:u8 = imu.get_gyro_settling();
                // while intflags != 0 {
                    for _ in 0..count {
                        if let Ok(sample) = imu.read_fifo_sample() {
                            if queue.enqueue(sample).is_err() {
                                debug!("on_fifo_interrupt enqueue error");
                            }
                        }
                    }
                    intflags_count = imu.fifo_samples_available().unwrap_or((0,0));
                    let (mut intflags, mut count) = intflags_count;
                    if intflags != 0 {
                        for _ in 0..count {
                            if let Ok(sample) = imu.read_fifo_sample() {
                                if queue.enqueue(sample).is_err() {
                                    debug!("on_fifo_interrupt enqueue error");
                                }
                            }
                        }
                    }
                // }
            });
        });
    }

    #[task(priority = 3, shared = [imu_stream, sample_consumer, filtered_acc, filtered_gyr, filtered_mag, filtered_temp, gyro_cal, acc_state, tx_prod, logging_state, tc])]
    async fn process_samples(mut cx: process_samples::Context) {
        let mut las: f32 = 0.0;
        let mut ars: f32 = 0.0;
        let mut current = ThreeAxes { x: 0.0, y: 0.0, z: 0.0 };
        let mut current_adjusted_temp:f32 = 0.0;


        // debug!("Processing samples...");

        // Get linear acceleration sensitivity
        cx.shared.imu_stream.lock(|imu| {
            match imu.get_linear_acc_sensitivty() {
                Ok(Some(sensitivity)) => las = sensitivity,
                Ok(None) => {
                    debug!("get_linear_acc_sensitivty not available (sensor off)");
                }
                Err(e) => {
                    debug!("Failed to read get_linear_acc_sensitivty: {:?}", e);
                }
            }
            match imu.get_angular_rate_sensitivty() {
                Ok(Some(sensitivity)) => ars = sensitivity,
                Ok(None) => {
                    debug!("get_angular_rate_sensitivty not available (sensor off)");
                }
                Err(e) => {
                    debug!("Failed to read get_angular_rate_sensitivty: {:?}", e);
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
                            // debug!("Accel, {:.3}, {:.3}, {:.3}", filtered_acc.data.x, filtered_acc.data.y, filtered_acc.data.z);
                            cx.shared.acc_state.lock(|mut acc_state| {
                                if !acc_state.initialized {
                                    acc_state.norm_last = calculate_sqrt(
                                        (filtered_acc.data.x*filtered_acc.data.x +
                                            filtered_acc.data.y*filtered_acc.data.y +
                                            filtered_acc.data.z*filtered_acc.data.z) as f64).unwrap();
                                    acc_state.initialized = true;
                                    // debug!("Accel initialized");
                                } else {
                                    let norm = calculate_sqrt((filtered_acc.data.x*filtered_acc.data.x +
                                                                        filtered_acc.data.y*filtered_acc.data.y +
                                                                        filtered_acc.data.z*filtered_acc.data.z) as f64).unwrap();
                                    let norm_diff = (norm - acc_state.norm_last).abs();
                                    // debug!("Accel change, {:.3}, {:.3}, {:.3}", norm_diff, norm, acc_state.norm_last);
                                    if norm_diff > 2.0 {
                                        // debug!("Accel change, {:.3}, {:.3}, {:.3}", filtered_acc.data.x, filtered_acc.data.y, filtered_acc.data.z);
                                        acc_state.is_stable = false;
                                        acc_state.stable_count = 0;
                                        acc_state.norm_last = norm;
                                    } else {
                                        if acc_state.stable_count < 52 { // ODR = 1 sec
                                            acc_state.stable_count += 1;
                                        } else {
                                            acc_state.is_stable = true;
                                        }

                                        // debug!("acc_state.is_stable {}", acc_state.is_stable);
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
                            // let x: f32 = gyro_temp_correction(gyro_data[0] as f32, filtered_temp.data, 13.55866667, current_adjusted_temp);
                            // let y: f32 = gyro_temp_correction(gyro_data[1] as f32, filtered_temp.data, 2.445666667, current_adjusted_temp);
                            // let z: f32 = gyro_temp_correction(gyro_data[2] as f32, filtered_temp.data, 3.157666667, current_adjusted_temp);
                            let mut x: f32 = gyro_data[0] as f32;
                            let mut y: f32 = gyro_data[1] as f32;
                            let mut z: f32 = gyro_data[2] as f32;
                            cx.shared.tc.lock(|tc| {
                                (x, y, z) = tc.correct(current_adjusted_temp, x, y, z);
                            });
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
                                                // debug!("Gyro offset, {:.3}, {:.3}, {:.3}", gyro_cal.data.x, gyro_cal.data.y, gyro_cal.data.z);
                                                // debug!("Gyro calibrated, {:.3}, {:.3}, {:.3}", cal_gyro.x, cal_gyro.y, cal_gyro.z);
                                                debug!("Gyro, {:.3}, {:.3}, {:.3}", filtered_gyr.data.x, filtered_gyr.data.y, filtered_gyr.data.z);
                                                cx.shared.logging_state.lock(|logging_state| {
                                                    if logging_state.is_csvlog_header_printed() {
                                                        let mut msg: HString<128> = HString::new();
                                                        let _ = write!(&mut msg, "{}, {:.3}, {:.3}, {:.3}\r\n", current_adjusted_temp, filtered_gyr.data.x, filtered_gyr.data.y, filtered_gyr.data.z);
                                                        log_str::spawn(msg).ok();
                                                    }
                                                });
                                            }
                                        });
                                    } else {
                                        // debug!("Gyro, {:.3}, {:.3}, {:.3}", filtered_gyr.data.x, filtered_gyr.data.y, filtered_gyr.data.z);
                                        gyro_cal.feed(filtered_gyr.data);
                                        // debug!("Gyro Cal Count: {}", gyro_cal.get_cal_count());
                                    }
                                });
                            });
                        });
                    }
                    FifoSample::Mag(mag_data) => {
                        debug!("Mag: {:?}", mag_data);
                    }
                    FifoSample::Temperature(temp_data) => {
                        let temp_c = (temp_data[0] as f32) / 256.0;
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

                        debug!("Temperature: {:#.6} degC", temp_c);
                        cx.shared.acc_state.lock(|acc_state| {
                            if acc_state.is_stable {
                                // Print filtered temperature when stable
                                // debug!("Temperature: {},", current_adjusted_temp);
                            }
                        });
                    }
                    FifoSample::Unknown(tag) => {
                        // debug!("Unknown sample type: {:#02X}", tag);
                    }
                }
            }
        });
    }

    /// USART1 interrupt handler: handles both RXNE and TXE servicing.
    #[task(binds = USART1, priority = 2, shared = [tx_prod, tx, tx_cons, logging_state], local = [rx, rx_prod])]
    fn usart1_irq(mut cx: usart1_irq::Context) {
        // RX path
        if let Ok(byte) = cx.local.rx.read() {
            let _ = cx.local.rx_prod.enqueue(byte);
            cx.shared.logging_state.lock(|logging_state| {
                if !logging_state.is_csvlog_header_printed() {
                    cx.shared.tx_prod.lock(|tx_prod| { // Echo back
                        if tx_prod.enqueue(byte).is_err() {
                            // Log or count failure
                            rprintln!("usart1_irq TX queue full, dropping byte!");
                        }
                    });
                }
            });
            if byte == b'\r' || byte == b'\n' {
                parse_rx::spawn().ok();
            }
        }

        // TX path
        cx.shared.tx_cons.lock(|tx_cons| {
            cx.shared.tx.lock(|tx| {
                if tx.is_tx_empty() {
                    if let Some(b) = tx_cons.dequeue() {
                        tx.write(b).ok();
                    } else {
                        tx.unlisten();
                    }
                }
            });
        });
    }

    /// Parse any received bytes into commands. Drains RX queue, accumulates a line, and executes.
    #[task(shared = [tx_prod, rx_cons, led, tx], priority = 6, local = [line_buf])]
    async fn parse_rx(mut cx: parse_rx::Context) {
        // Drain RX queue
        loop {
            let b_opt = cx.shared.rx_cons.lock(|c| c.dequeue());
            let b = match b_opt { Some(b) => b, None => break };


            // Treat CR/LF as end of command
            if b == b'\r' || b == b'\n' {
                if cx.local.line_buf.is_empty() {
                    continue; // ignore empty line
                }
                // Execute command
                let cmd_buf = core::mem::take(cx.local.line_buf);
                exec_cmd::spawn(cmd_buf.as_str().parse().unwrap()).ok();
                // // Prompt
                // cx.shared.tx_prod.lock(|tx_prod| {
                //         send_to_uart(tx_prod, &"> ".as_bytes());
                // });
                // Kick TX
                cortex_m::peripheral::NVIC::pend(USART1_IRQ);
            } else {
                if cx.local.line_buf.len() < LINE_BUF_SIZE {
                    cx.local.line_buf.push(b as char).ok();
                } else {
                    cx.shared.tx_prod.lock(|tx_prod| {
                            enqueue_tx_bytes(tx_prod, b"\r\nERR: line too long\r\n");
                    });
                    cx.local.line_buf.clear();
                }
            }
            cx.shared.tx.lock(|tx| {
                tx.listen(); // enables TXE interrupt
            });
        }
    }

// ===== Helpers =====

    /// Minimal command executor.
    #[task(shared = [tx_prod, tx_cons, tx, imu_stream, gyro_cal, logging_state], priority = 7)]
    async fn exec_cmd(mut cx: exec_cmd::Context, mut cmd: HString<64>) {
        cx.shared.tx_prod.lock(|tx_prod| {
            match cmd.as_str().trim() {
                "help" => {
                    send_to_uart(
                        tx_prod,
                        &"\r\nCommands:\r\n help - this text\r\n recal - recalibrate gyro\r\n stop debug\r\n csv log\r\n kick imu - restarts fifo read\r\n".as_bytes(),
                    );
                }
                "recal" => {
                    cx.shared.gyro_cal.lock(|gyro_cal| {
                        gyro_cal.reset_calibration();
                    });
                    send_to_uart(
                        tx_prod,
                        &"\r\nrecal started\r\n".as_bytes());
                }
                "debug" => unsafe {
                    if crate::DEBUG_LOG_ACTIVE {
                        crate::DEBUG_LOG_ACTIVE = false;
                        rprintln!("debug logging stopped");
                    } else {
                        crate::DEBUG_LOG_ACTIVE = true;
                        rprintln!("debug logging started");
                    }
                }
                "csv on" => unsafe {
                    if !crate::DEBUG_LOG_ACTIVE {
                        cx.shared.logging_state.lock(|logging_state| {
                            if logging_state.is_csvlog_header_printed() {
                                logging_state.set_csvlog_header_printed(false);
                                rprintln!("csv logging stopped");
                            } else {
                                send_to_uart(
                                    tx_prod,
                                    &"temperature, gx, gy, gz\r\n".as_bytes());

                                rprintln!("csv logging started");
                                logging_state.set_csvlog_header_printed(true);
                            }
                        });
                    } else {
                        rprintln!("csv log requires debug logging to be disabled use 'debug' to toggle off");
                    }
                }
                "csv off" => {
                    cx.shared.logging_state.lock(|logging_state| {
                        logging_state.set_csvlog_header_printed(false);
                    });
                }
                "kick imu" => {
                    let mut intflags_samples:(u8, u16) = (0, 0);
                    cx.shared.imu_stream.lock(|imu| {
                        intflags_samples = imu.fifo_samples_available().unwrap();
                        imu.dump_fifo();
                    });
                    let (intflags, samples) = intflags_samples;

                    let mut resp: Vec<u8, 32> = Vec::new();
                    let _ = write!(resp, "\r\nimu kicked: {:#02X}:{}\r\n", intflags, samples);
                    send_to_uart(tx_prod, &resp.as_slice());
                }
                other => {
                    cx.shared.logging_state.lock(|logging_state| {
                        if !logging_state.is_csvlog_header_printed() {
                            let mut resp: Vec<u8, 64> = Vec::new();
                            let _ = write!(resp, "\r\nERR: unknown cmd: {}\r\n", other);
                            send_to_uart(tx_prod, &resp.as_slice());
                        }
                    });
                }
            }
        });

        // Ensure TX starts or keeps going
        cortex_m::peripheral::NVIC::pend(Interrupt::USART1);
        cx.shared.tx.lock(|tx| {
            if tx.is_tx_empty() {
                cx.shared.tx_cons.lock(|tx_cons| {
                    if !tx_cons.peek().is_none() { // if there is data to send
                        tx.listen();
                    }
                });
            }
        });
    }


    fn gyro_temp_correction(gyro_data: f32, temp: f32, factor: f32, temperature: f32) -> f32 {
        // Reference temperature (°C) at which the gyro was calibrated
        const REF_TEMP_C: f32 = 0.0;
        // Temperature drift slope (units of gyro_data per °C).
        // Adjust this based on your sensor's characterization.
        // const SLOPE: f32 = 0.7;
        //
        // let delta_t = temp - REF_TEMP_C;
        let adjustment = temperature * factor;

        // Apply correction: if your measured drift increases with temperature,
        // subtract the adjustment; if it decreases, flip the sign or SLOPE.
        gyro_data - adjustment
    }

    fn parse_fifo_frame(frame: &[u8; 7]) {
        let tag = frame[0];
        let x = i16::from_le_bytes([frame[1], frame[2]]);
        let y = i16::from_le_bytes([frame[3], frame[4]]);
        let z = i16::from_le_bytes([frame[5], frame[6]]);

        // debug!("Tag: 0x{:02X}, X: {}, Y: {}, Z: {}", tag, x, y, z);
    }

    fn scan_i2c_bus<I2C, E>(i2c: &mut I2C)
    where
        I2C: I2cTrait<Error = E>,
        E: core::fmt::Debug,
    {
        debug!("Scanning I2C bus...");
        for addr in 0x08..=0x77 {
            let mut buf = [0u8];
            match i2c.write_read(addr, &[0x00], &mut buf) {
                Ok(_) => debug!(" - Found device at 0x{:02X}", addr),
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

    #[idle(shared = [sample_consumer, delay, int1_pin, imu_stream], local = [last_active_int1_time])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            ctx.shared.int1_pin.lock(|pin| {
                let is_high = pin.is_high().unwrap_or(false);

                match (is_high, ctx.local.last_active_int1_time.as_ref()) {
                    (true, None) => {
                        *ctx.local.last_active_int1_time = Some(Mono::now());
                    }
                    (true, Some(last_time)) => {
                        let elapsed = Mono::now() - *last_time;
                        rprintln!("INT1 active for {} ms", elapsed.to_millis());

                        if elapsed.to_millis() > 100 {
                            *ctx.local.last_active_int1_time = None;
                            ctx.shared.imu_stream.lock(|imu| imu.dump_fifo());
                            rprintln!("INT1 stuck --> fifo dumped");
                        }
                    }
                    (false, _) => {
                        *ctx.local.last_active_int1_time = None;
                    }
                }
            });

            let queue_len = ctx.shared.sample_consumer.lock(|c| c.len());
            if queue_len > 0 {
                process_samples::spawn().ok();
            } else {
                ctx.shared.delay.lock(|d| d.delay_ms(10_u32));
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

    fn send_to_uart(tx_prod: &mut Producer<'static, u8, TX_QSIZE>, bytes: &[u8]) {
        for &b in bytes {
            if tx_prod.enqueue(b).is_err() {
                // Log or count failure
                rprintln!("send_to_uart TX queue full, dropping byte!");
            }
        }
    }

}