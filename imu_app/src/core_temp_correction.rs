pub struct TempCompensator {
    odr: f32,       // Hz
    prev_temp: f32, // last temperature
    first: bool,    // flag to skip derivative on first sample
}

impl TempCompensator {
    pub fn new(odr: f32) -> Self {
        Self {
            odr,
            prev_temp: 0.0,
            first: true,
        }
    }

    pub fn correct(&mut self, temp: f32, x_raw: f32, y_raw: f32, z_raw: f32) -> (f32, f32, f32) {
        // compute dT/dt
        let dtdt = if self.first {
            self.first = false;
            0.0
        } else {
            (temp - self.prev_temp) * self.odr
        };
        self.prev_temp = temp;

        // coeffs (from your fit)
        let (mx, kx, bx) = (13.693612, 26.446959, 82.489701);
        let (my, ky, by) = (2.622601, 15.180464, -60.567579);
        let (mz, kz, bz) = (4.004207, 113.460081, -47.904750);

        // bias per axis
        let bias_x = mx * temp + kx * dtdt + bx;
        let bias_y = my * temp + ky * dtdt + by;
        let bias_z = mz * temp + kz * dtdt + bz;

        // corrected values
        (
            x_raw - bias_x,
            y_raw - bias_y,
            z_raw - bias_z,
        )
    }
}
