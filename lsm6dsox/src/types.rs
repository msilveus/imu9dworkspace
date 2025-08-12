#[derive(Debug, Clone, Copy)]
pub enum FifoSample {
    Accel([i16; 3]),
    Gyro([i16; 3]),
    Mag([i16; 3]), // LIS3MDL via sensor hub
    Temperature([i16; 1]), // Temperature
    Unknown(u8),   // For unexpected tag handling
}
