use libm::{atan2f, powf, sqrtf};

#[derive(Debug, Clone, Copy)]
pub struct Vector {
    pub magnitude: f32,
    pub phase: f32,
}

impl Vector {
    pub fn from_cartesian(x: f32, y: f32) -> Self {
        Vector {
            magnitude: sqrtf(powf(x, 2.) + powf(y, 2.)),
            phase: atan2f(y, x),
        }
    }
    pub fn clamp_magnitude(self, limit: f32) -> Self {
        Vector {
            magnitude: self.magnitude.clamp(-limit, limit),
            ..self
        }
    }
    pub fn rotated(self, angle: f32) -> Self {
        Vector {
            phase: self.phase + angle,
            ..self
        }
    }
}
