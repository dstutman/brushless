#[derive(Clone, Copy, Debug)]
pub struct Constants {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Maximum magnitude of the accumulator, used for anti-windup
    pub la: f32,
}

impl Constants {
    pub fn new(kp: f32, ki: f32, kd: f32, la: f32) -> Self {
        Constants { kp, ki, kd, la }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Pid {
    constants: Constants,
    accumulator: f32,
    // Previous value of the process variable
    previous: Option<f32>,
    // The output calculated from the last update
    output: f32,
}

impl Pid {
    /// `integral_limit`: The maximum magnitude of the integral (anti-windup)
    pub fn new(constants: Constants) -> Self {
        Pid {
            constants,
            accumulator: 0.,
            previous: None,
            output: 0.,
        }
    }

    pub fn update(&mut self, val: f32, setpoint: f32, dt: f32) {
        if val.is_subnormal() || dt.is_subnormal() {
            log::warn!("Subnormal argument, val was: {}, dt was: {} ", val, dt);
            return;
        }

        let Constants { kp, ki, kd, la } = self.constants;

        let err = setpoint - val;
        self.accumulator += err * dt;
        if self.accumulator.abs() > la {
            log::debug!("Accumulator limiter triggered");
            self.accumulator = self.accumulator.clamp(-la, la);
        }
        let derivative = if let Some(previous) = self.previous {
            (val - previous) / dt
        } else {
            0.
        };

        self.previous = Some(val);

        self.output = kp * err + ki * self.accumulator + kd * derivative;
    }

    pub fn output(&self) -> f32 {
        self.output
    }
}
