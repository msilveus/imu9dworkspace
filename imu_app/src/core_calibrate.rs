use rtt_target::rprintln;
use crate::core_data_types::ThreeAxes;

#[derive(Debug)]
pub struct CalibratedGyro {
    pub data: ThreeAxes,
    pub initialized: bool,
    pub is_calibrated: bool,
    delta_threshold: f32,
    required_samples: usize,
    sample_sum: ThreeAxes,
    sample_count: usize,
    last_sample: Option<ThreeAxes>,
}

impl CalibratedGyro {
    pub fn new(delta_threshold: f32, required_samples: usize) -> Self {
        Self {
            data: ThreeAxes::default(),
            initialized: false,
            is_calibrated: false,
            delta_threshold,
            required_samples,
            sample_sum: ThreeAxes::default(),
            sample_count: 0,
            last_sample: None,
        }
    }

    pub fn init(&mut self) {
        self.data = ThreeAxes::default();
        self.initialized = false;
        self.is_calibrated = false;
        self.sample_sum = ThreeAxes::default();
        self.sample_count = 0;
        self.last_sample = None;
    }

    pub fn feed(&mut self, new_sample: ThreeAxes) {
        if self.is_calibrated {
            return;
        }

        if !self.initialized {
            self.last_sample = Some(new_sample);
            self.initialized = true;
            return;
        }

        let last = self.last_sample.unwrap();
        if last.is_within_delta(&new_sample, self.delta_threshold) {
            self.sample_sum.add(&new_sample);
            self.sample_count += 1;

            if self.sample_count >= self.required_samples {
                let mut avg = self.sample_sum;
                avg.scale(self.sample_count as f32);
                self.data = avg;
                self.is_calibrated = true;
            }
        } else {
            // Not stable — reset
            self.sample_sum = ThreeAxes::default();
            self.sample_count = 0;
            self.last_sample = Some(new_sample);
        }
    }

    pub fn reset_calibration(&mut self) {
        self.init();
        rprintln!("Resetting Gyro calibration");
    }

    pub fn get_cal_count(&self) -> usize {
        self.sample_count
    }
}