#[derive(Debug, Clone, Copy, Default)]
pub struct ThreeAxes {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl ThreeAxes {
    pub fn is_within_delta(&self, other: &ThreeAxes, delta: f32) -> bool {
        (self.x - other.x).abs() <= delta &&
            (self.y - other.y).abs() <= delta &&
            (self.z - other.z).abs() <= delta
    }

    pub fn add(&mut self, other: &ThreeAxes) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }

    pub fn scale(&mut self, factor: f32) {
        self.x /= factor;
        self.y /= factor;
        self.z /= factor;
    }
}
