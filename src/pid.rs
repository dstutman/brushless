#[derive(Clone, Copy, Debug)]
pub struct Constants {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

impl Constants {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Constants { kp, ki, kd }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Pid {
    constants: Constants,
    accumulator: f32,
    // Previous value of the process variable
    previous: f32,
    // The output calculated from the last update
    output: f32,
}

impl Pid {
    pub fn new(constants: Constants) -> Self {
        Pid {
            constants,
            accumulator: 0.,
            previous: 0.,
            output: 0.,
        }
    }

    pub fn update(&mut self, val: f32, setpoint: f32, dt: f32) {
        let err = setpoint - val;
        // TODO: Anti-windup
        self.accumulator += err;
        // TODO: Guard against very small dt
        let derivative = (val - self.previous) / dt;
        let Constants { kp, ki, kd } = self.constants;
        self.output = kp * err + ki * self.accumulator + kd * derivative;
    }

    pub fn output(&self) -> f32 {
        self.output
    }
}
