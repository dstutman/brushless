//! Implements field oriented regulation using direct current and angle sensing.
use libm::{cosf, fabsf, sinf, sqrtf};

use crate::pid::{Constants, Pid};
use crate::util::Vector;

#[derive(Debug, Clone, Copy)]
pub struct MotorProperties {
    /// Kilogram meters-squared
    pub inertia: f32,
    pub pole_pairs: usize,
    /// Ohms
    pub phase_resistance: f32,
    /// Newton meters per amp
    pub torque_constant: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum Target {
    /// Newton meters
    Torque(f32),
    /// Radians per second
    Velocity(f32),
    /// Radians
    /// Be careful when switching into position mode with a
    /// rotation sensor that supports tracking multiple rotations
    /// (positions outside of the range [0..2PI]). You may be at some huge
    Position(f32),
}

#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// Newton meters
    pub torque_magnitude: Option<f32>,
    /// Radians per second
    pub speed: Option<f32>,
    /// Radians, first to second CCW, inclusive.
    /// Values may exceed 2PI to allow multiple
    /// rotations.
    pub position: Option<(f32, f32)>,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            torque_magnitude: None,
            speed: None,
            position: None,
        }
    }
}

/// Amps
#[derive(Debug, Clone, Copy)]
pub struct PhaseCurrents {
    pub u: f32,
    pub v: f32,
    pub w: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct State {
    /// Microseconds, wrapping allowed
    pub timestamp_us: u32,
    /// Radians
    pub position: f32,
    /// Radians per second
    pub velocity: f32,
    pub currents: PhaseCurrents,
}

#[derive(Debug, Clone, Copy)]
struct ClarkeCurrent {
    alpha: f32,
    beta: f32,
}

impl From<PhaseCurrents> for ClarkeCurrent {
    fn from(currents: PhaseCurrents) -> Self {
        ClarkeCurrent {
            alpha: currents.u,
            beta: 1. / sqrtf(3.) * (currents.v - currents.w),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ParkCurrents {
    direct: f32,
    quadrature: f32,
}

/// Rotates the currents into the rotor frame, `angle` in radians
fn park_transform(currents: ClarkeCurrent, angle: f32) -> ParkCurrents {
    // TODO: Check if LLVM would do this automatically
    let sin = sinf(angle);
    let cos = cosf(angle);
    ParkCurrents {
        direct: cos * currents.alpha + sin * currents.beta,
        quadrature: -sin * currents.alpha + cos * currents.beta,
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Status {
    pub torque_limited: bool,
    pub speed_limited: bool,
    pub position_limited: bool,
}

#[derive(Debug, Clone)]
pub struct Regulator {
    motor: MotorProperties,
    target: Target,
    limits: Limits,
    // Current -> Duty cycle
    dcurrent_pid: Pid,
    // Current -> Duty cycle
    qcurrent_pid: Pid,
    velocity_pid: Pid,
    position_pid: Pid,
    // Records the timestamp of `state` at the previous update.
    prev_us: Option<u32>,
    output: Vector,
    status: Status,
}

impl Regulator {
    pub fn update(&mut self, state: State) {
        log::info!("Regulating for: {:?}, to: {:?}", state, self.target);
        let dt = 1E-6
            * if let Some(prev_us) = self.prev_us {
                // Handle a wrapping timestamp
                if state.timestamp_us < prev_us {
                    log::debug!("Timestamp wrapped");
                    u32::MAX - prev_us + state.timestamp_us
                } else {
                    state.timestamp_us - prev_us
                }
            } else {
                0
            } as f32;
        self.prev_us = Some(state.timestamp_us);

        // Phi is the electrical angle
        let phi = state.position * self.motor.pole_pairs as f32;
        let park = park_transform(ClarkeCurrent::from(state.currents), phi);

        // Regulate currents to required targets
        // TODO: Implement field weakening for high speed operation.
        let target_torque = match self.target {
            Target::Torque(target_torque) => target_torque,
            Target::Velocity(target_velocity) => {
                self.velocity_pid
                    .update(state.velocity, target_velocity, dt);
                self.velocity_pid.output()
            }
            Target::Position(target_position) => {
                // NOTE: Maybe cascade the position loop with the velocity loop
                self.position_pid
                    .update(state.position, target_position, dt);
                self.position_pid.output()
            }
        };

        // Reset regulator status
        self.status = Status::default();
        // The ordering of these limit checks imposes a priority.
        // Torque is the highest priority limit, then velocity, and then position.
        // Implements an aggressive limiting scheme that applies full torque towards
        // desired operating region. This almost certainly saturates the integral and
        // derivative terms of the current regulators.
        log::debug!("Limits are: {:?}", self.limits);
        let target_torque = if let Some((lower_limit, upper_limit)) = self.limits.position {
            if state.position < lower_limit {
                log::info!("Position limiter triggered");
                self.status.position_limited = true;
                f32::MAX
            } else if state.position > upper_limit {
                log::info!("Position limiter triggered");
                self.status.position_limited = true;
                f32::MIN
            } else {
                target_torque
            }
        } else {
            target_torque
        };
        let target_torque = if let Some(limit) = self.limits.speed {
            if fabsf(state.velocity) > limit {
                log::info!("Speed limiter triggered");
                self.status.speed_limited = true;
                if state.velocity.is_sign_positive() {
                    f32::MIN
                } else {
                    f32::MAX
                }
            } else {
                target_torque
            }
        } else {
            target_torque
        };
        let target_torque = if let Some(limit) = self.limits.torque_magnitude {
            if fabsf(target_torque) > limit {
                log::info!("Torque limiter triggered");
                self.status.torque_limited = true;
                target_torque.clamp(-limit, limit)
            } else {
                target_torque
            }
        } else {
            target_torque
        };
        log::debug!("Target torque: {}", target_torque);

        self.dcurrent_pid.update(park.direct, 0., dt);
        self.qcurrent_pid.update(
            park.quadrature,
            1. / self.motor.torque_constant * target_torque,
            dt,
        );

        // Our output is a duty cycle vector in the stator fixed frame
        let duty_vector =
            Vector::from_components(self.dcurrent_pid.output(), self.qcurrent_pid.output());
        // Inverse Park transform, positive rotation of vector
        // by phi corresponds to negative rotation of frame
        // (moving back to stator fixed)
        self.output = duty_vector.rotated(phi);

        log::debug!("Output is: {:?}", self.output);
    }
    pub fn set_target(&mut self, target: Target) {
        self.target = target;
    }
    pub fn set_limits(&mut self, limits: Limits) {
        self.limits = limits;
    }
    pub fn output(&self) -> Vector {
        self.output
    }
    pub fn status(&self) -> Status {
        self.status
    }
}

#[derive(Debug, Clone)]
pub struct RegulatorConfig {
    motor: MotorProperties,
    limits: Limits,
    dcurrent_consts: Constants,
    qcurrent_consts: Constants,
    velocity_consts: Constants,
    position_consts: Constants,
}

impl RegulatorConfig {
    pub fn new(motor: MotorProperties) -> Self {
        RegulatorConfig {
            motor,
            limits: Limits {
                torque_magnitude: None,
                speed: None,
                position: None,
            },
            // TODO: Intelligent selection of constants
            // NOTE: Selection will require adequate scaling
            // to account for expected setpoint range.
            dcurrent_consts: Constants {
                kp: 1.,
                ki: 0.,
                kd: 0.,
                la: f32::MAX,
            },
            qcurrent_consts: Constants {
                kp: 1.,
                ki: 0.,
                kd: 0.,
                la: f32::MAX,
            },
            velocity_consts: Constants {
                kp: 1.,
                ki: 0.,
                kd: 0.,
                la: f32::MAX,
            },
            position_consts: Constants {
                kp: 1.,
                ki: 0.,
                kd: 0.,
                la: f32::MAX,
            },
        }
    }
    /// Realize a `Regulator` from the configuration
    pub fn realize(&mut self) -> Regulator {
        Regulator {
            motor: self.motor,
            target: Target::Torque(0.),
            limits: self.limits,
            dcurrent_pid: Pid::new(self.dcurrent_consts),
            qcurrent_pid: Pid::new(self.qcurrent_consts),
            velocity_pid: Pid::new(self.velocity_consts),
            position_pid: Pid::new(self.position_consts),
            prev_us: None,
            output: Vector::new(0., 0.),
            status: Status::default(),
        }
    }
    pub fn set_velocity_consts(&mut self, velocity_consts: Constants) -> &mut Self {
        self.velocity_consts = velocity_consts;
        self
    }
    pub fn set_position_consts(&mut self, position_consts: Constants) -> &mut Self {
        self.position_consts = position_consts;
        self
    }
    pub fn set_dcurrent_consts(&mut self, dcurrent_consts: Constants) -> &mut Self {
        self.dcurrent_consts = dcurrent_consts;
        self
    }
    pub fn set_qcurrent_consts(&mut self, qcurrent_consts: Constants) -> &mut Self {
        self.qcurrent_consts = qcurrent_consts;
        self
    }
}
