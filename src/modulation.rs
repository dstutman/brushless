use core::{
    f32::consts::PI,
    ops::{Add, Mul},
};

use crate::util::Vector;

/// Duty cycles per phase, [0..1]
pub struct PhaseDuties {
    pub u: f32,
    pub v: f32,
    pub w: f32,
}

impl PhaseDuties {
    pub fn new(u: f32, v: f32, w: f32) -> Self {
        PhaseDuties { u, v, w }
    }
}

impl Add for PhaseDuties {
    type Output = PhaseDuties;
    fn add(self, rhs: Self) -> Self::Output {
        PhaseDuties {
            u: self.u + rhs.u,
            v: self.v + rhs.v,
            w: self.w + rhs.w,
        }
    }
}

impl Mul<f32> for PhaseDuties {
    type Output = PhaseDuties;
    fn mul(self, rhs: f32) -> Self::Output {
        PhaseDuties {
            u: self.u * rhs,
            v: self.v * rhs,
            w: self.w * rhs,
        }
    }
}

/// Implements Space Vector (Pulse-Width) modulation.
// `rustfmt` formats the match arms really poorly, so we disable it here
#[rustfmt::skip]
pub fn space_vector_modulation(vector: Vector) -> PhaseDuties {
    log::info!("Modulating for: {:?}", vector);
    if vector.magnitude > 1. {log::warn!("Space Vector saturated driver, magntiude: {}", vector.magnitude)}
    let Vector {
        magnitude, phase, ..
    } = vector.clamped_magnitude(1.);
    // Convert the phase from radians to sector-counts
    let sectors = phase / (2. * PI / 6.);
    // Re-range the sector-counts to one full rotation around the
    // space-vector circle
    let base_sectors = sectors % 6.;
    // Correct the sector signs
    let sector = if base_sectors.is_sign_positive() {
        base_sectors
    } else {
        base_sectors + 6.
    };

    let sector_index = sector as u32;
    // `f32::fract(self)` is only available in `std` so
    // do the same with mod
    let mixing_fraction = sector % 1.;

    // Some opportunities for optimization here by reordering multiplications.
    // Can reduce switching losses by being smarter with zero-vector selection.
    match sector_index {
        // A wrap around to 6 is possible due to floating point negative zero
        0 | 6 => {
            PhaseDuties {
                u: 1.,
                v: 0.,
                w: 0.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 1.,
                v: 1.,
                w: 0.,
            } * (1. - mixing_fraction) * magnitude
        }
        1 => {
            PhaseDuties {
                u: 1.,
                v: 1.,
                w: 0.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 0.,
                v: 1.,
                w: 0.,
            } * (1. - mixing_fraction) * magnitude
        }
        2 => {
            PhaseDuties {
                u: 0.,
                v: 1.,
                w: 0.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 0.,
                v: 1.,
                w: 1.,
            } * (1. - mixing_fraction) * magnitude
        }
        3 => {
            PhaseDuties {
                u: 0.,
                v: 1.,
                w: 1.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 0.,
                v: 0.,
                w: 1.,
            } * (1. - mixing_fraction) * magnitude
        }
        4 => {
            PhaseDuties {
                u: 0.,
                v: 0.,
                w: 1.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 1.,
                v: 0.,
                w: 1.,
            } * (1. - mixing_fraction) * magnitude
        }
        5 => {
            PhaseDuties {
                u: 1.,
                v: 0.,
                w: 1.,
            } * mixing_fraction * magnitude
            + PhaseDuties {
                u: 0.,
                v: 0.,
                w: 1.,
            } * (1. - mixing_fraction) * magnitude
        }
        _ => {
            log::error!("Invalid modulation sector index: {}", sector_index);
            unreachable!();
        }
    }
}
