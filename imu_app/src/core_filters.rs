use crate::core_data_types::ThreeAxes;

#[derive(Debug, Clone)]
pub struct FilteredAxes {
    pub data: ThreeAxes,
    pub initialized: bool,
}

#[derive(Debug, Clone)]
pub struct FilteredTemp {
    pub data: f32,
    pub initialized: bool,
}

/// Applies a low-pass filter to a single float value.
/// `current` is the new sample, `filtered` is the previous filtered value (will be updated),
/// and `alpha` is the blending factor in the range [0.0, 1.0].
/// Applies a low-pass filter to a single float.
pub fn low_pass_filter_float(current: f32, previous: f32, alpha: f32) -> f32 {
    let alpha = alpha.clamp(0.0, 1.0);
    alpha * current + (1.0 - alpha) * previous
}

/// Applies a low-pass filter to 3 axes with initialization logic.
pub fn low_pass_filter_3axes(
    current: &ThreeAxes,
    filtered: &mut FilteredAxes,
    alpha: f32,
) {
    if !filtered.initialized {
        // First-time setup: directly copy the current value
        filtered.data = *current;
        filtered.initialized = true;
    } else {
        filtered.data.x = low_pass_filter_float(current.x, filtered.data.x, alpha);
        filtered.data.y = low_pass_filter_float(current.y, filtered.data.y, alpha);
        filtered.data.z = low_pass_filter_float(current.z, filtered.data.z, alpha);
    }
}
