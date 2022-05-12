use core::f32::consts::PI;

use libm::{atan2f, fabsf, powf, sqrtf};

#[derive(Debug, Clone, Copy)]
pub struct Vector {
    pub magnitude: f32,
    pub phase: f32,
    // Block construction so magnitude invariants can be upheld
    _private: (),
}

impl Vector {
    pub fn new(magnitude: f32, phase: f32) -> Vector {
        Vector {
            magnitude: fabsf(magnitude),
            phase: if magnitude.is_sign_positive() {
                phase
            } else {
                phase + PI
            },
            _private: (),
        }
    }
    pub fn from_components(x: f32, y: f32) -> Self {
        Vector {
            magnitude: sqrtf(powf(x, 2.) + powf(y, 2.)),
            phase: atan2f(y, x),
            _private: (),
        }
    }
    pub fn clamped_magnitude(self, limit: f32) -> Self {
        Vector {
            magnitude: self.magnitude.clamp(0., limit),
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
