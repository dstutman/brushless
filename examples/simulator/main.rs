use std::num::Wrapping;

use eframe;

mod gui;
mod physics;

use brushless::{
    modulation::space_vector_modulation,
    regulator::{Limits, MotorProperties, Regulator, RegulatorConfig, State, Target},
};
use gui::{Gui, TargetType};
use physics::RotorSimulator;

struct App {
    time_us: Wrapping<u32>,
    gui: Gui,
    rotor: RotorSimulator,
    regulator: Regulator,
    last_targettype: TargetType,
}

impl App {
    pub fn default() -> Self {
        let motor = MotorProperties {
            inertia: 1E-4,
            pole_pairs: 7,
            phase_resistance: 22.,
            torque_constant: 0.05,
        };
        App {
            time_us: Wrapping(0),
            gui: Gui::new(),
            rotor: RotorSimulator::new(motor, 3.7 * 4.),
            regulator: RegulatorConfig::new(motor).realize(),
            last_targettype: TargetType::Torque,
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &eframe::egui::Context, _: &mut eframe::Frame) {
        let state = State {
            currents: self.rotor.phase_currents(),
            position: self.rotor.angle(),
            velocity: self.rotor.angular_velocity(),
            timestamp_us: self.time_us.0,
        };

        if !self.gui.paused {
            // If we aren't paused, keep repainting so the simulation/animations can run.
            ctx.request_repaint();
            let dt = self.gui.dilation * ctx.input().predicted_dt;
            self.time_us += (1E6 * dt) as u32;

            self.regulator.update(state);

            self.rotor
                .set_phase_duties(space_vector_modulation(self.regulator.output()));

            self.rotor.step(dt);
        }

        self.gui
            .update(state, self.regulator.status(), self.regulator.output());
        self.gui.show(ctx);

        let target = match self.gui.targets.selected {
            TargetType::Torque => Target::Torque(self.gui.targets.torque),
            TargetType::Velocity => Target::Velocity(self.gui.targets.velocity),
            TargetType::Position => Target::Position(self.gui.targets.position),
        };
        self.regulator.set_target(target);
        self.last_targettype = self.gui.targets.selected;
        if self.last_targettype != self.gui.targets.selected {
            self.regulator.reset_pids();
        }

        let limits = Limits::new(
            if self.gui.torque_limit.enabled {
                Some(self.gui.torque_limit.value)
            } else {
                None
            },
            if self.gui.speed_limit.enabled {
                Some(self.gui.speed_limit.value)
            } else {
                None
            },
            if self.gui.position_limits.enabled {
                let limits = self.gui.position_limits.value;
                // Reorder the user limits to uphold invariants
                // see `Limits::new`
                if limits.0 <= limits.1 {
                    Some(limits)
                } else {
                    Some((limits.1, limits.0))
                }
            } else {
                None
            },
        );
        self.regulator.set_limits(limits);

        self.rotor
            .set_perturbation_torque(self.gui.perturbation_torque);

        if self.gui.perturbation_impulse.triggered {
            self.rotor
                .apply_impulse(self.gui.perturbation_impulse.value)
        };

        if self.gui.reset_requested {
            self.rotor.reset();
            self.regulator.reset_pids();
            self.time_us = Wrapping(0);
        }
    }
}

fn main() {
    env_logger::init();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Brushless Simulator",
        native_options,
        Box::new(|_| Box::new(App::default())),
    );
}
