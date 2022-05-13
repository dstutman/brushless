use core::ops::Mul;

use libm::fabsf;

#[derive(Clone, Copy, Debug)]
pub struct Constants {
    /// Proportional coefficient
    pub kp: f32,
    /// Integral coefficient
    pub ki: f32,
    /// Derivative coefficient
    pub kd: f32,
}

/// All limits are on magnitude after scaling by kp, ki, or kd
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Proportional limit
    pub lp: f32,
    /// Integral limit
    pub li: f32,
    /// Derivative limit
    pub ld: f32,
    /// Output limit
    pub lo: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            lp: f32::MAX,
            li: f32::MAX,
            ld: f32::MAX,
            lo: f32::MAX,
        }
    }
}
#[derive(Debug, Clone, Copy)]
pub struct Output {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl Output {
    pub fn total(&self) -> f32 {
        self.p + self.i + self.d
    }
}

impl Mul<f32> for Output {
    type Output = Output;

    fn mul(self, rhs: f32) -> Self {
        Output {
            p: self.p * rhs,
            i: self.i * rhs,
            d: self.d * rhs,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pid {
    constants: Constants,
    limits: Limits,
    integral: f32,
    // Previous value of the process variable
    previous: Option<f32>,
    // The output calculated from the last update
    pub output: Output,
}

impl Pid {
    /// `integral_limit`: The maximum magnitude of the integral (anti-windup)
    pub fn new(constants: Constants, limits: Limits) -> Self {
        Pid {
            constants,
            limits,
            integral: 0.,
            previous: None,
            output: Output {
                p: 0.,
                i: 0.,
                d: 0.,
            },
        }
    }

    pub fn update(&mut self, val: f32, target: f32, dt: f32) {
        if dt == 0. || dt == -0. {
            log::warn!("Invalid dt argument, dt was: {} ", dt);
            return;
        }

        let Constants { kp, ki, kd } = self.constants;
        let Limits { lp, li, ld, lo } = self.limits;

        let err = target - val;

        let proportional = kp * err;
        let proportional = if fabsf(proportional) > lp {
            log::debug!("Proportional limiter triggered");
            if proportional.is_sign_positive() {
                lp
            } else {
                -lp
            }
        } else {
            proportional
        };
        log::debug!("Error: {}", err);

        // We only update the integral accumulator if the integral term is not
        // being limited. This avoids response lags when recovering from a
        // limited integral. This is different than the anti-windup integral
        // limited. We need to check >= because our total output limiter may
        // not be perfect due to floating point errors.
        if !(fabsf(self.output.total()) >= lo) {
            self.integral += ki * err * dt;
            // Could use `clamp` but then we can't log
            if fabsf(self.integral) > li {
                log::debug!("Integral limiter triggered");
                self.integral = if self.integral.is_sign_positive() {
                    li
                } else {
                    -li
                }
            }
        } else {
            log::debug!("Output saturated, not updating integral");
        }

        // Avoid kick by using derivative of value, not error
        let derivative = if let Some(previous) = self.previous {
            // If less than target and increasing: derivative of error is negative
            // If less than target and decreasing: derivative of error is positive
            // If greater than target and increasing: derivative of error is positive
            // If greater than target and decreasing: derivative of error is negative
            if err.is_sign_positive() {
                -kd * (val - previous) / dt
            } else {
                kd * (val - previous) / dt
            }
        } else {
            0.
        };

        // Could use `clamp` but then we can't log
        let derivative = if fabsf(derivative) > ld {
            log::debug!("Derivative limiter triggered");
            if derivative.is_sign_positive() {
                ld
            } else {
                -ld
            }
        } else {
            derivative
        };

        let output = Output {
            p: proportional,
            i: self.integral,
            d: derivative,
        };
        let abs_total = fabsf(output.total());
        self.output = if abs_total > lo {
            log::debug!("Output limiter triggered");
            output * (lo / abs_total)
        } else {
            output
        };

        self.previous = Some(val);
    }

    pub fn reset(&mut self) {
        self.integral = 0.;
        self.previous = None;
        self.output = Output {
            p: 0.,
            i: 0.,
            d: 0.,
        };
    }
}
