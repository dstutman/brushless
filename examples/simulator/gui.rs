use std::f32::consts::PI;

use eframe::egui::{
    plot::{Arrows, Plot, Value, Values},
    *,
};

use brushless::regulator::{PhaseCurrents, State, Status};
use brushless::Vector;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TargetType {
    Torque,
    Velocity,
    Position,
}

#[derive(Debug, Clone, Copy)]
pub struct Targets {
    pub torque: f32,
    pub velocity: f32,
    pub position: f32,
    pub selected: TargetType,
}

#[derive(Debug, Clone, Copy)]
pub struct ToggleValue<T> {
    pub value: T,
    pub enabled: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct TriggerValue<T> {
    pub value: T,
    pub triggered: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct Gui {
    pub reset_requested: bool,
    pub paused: bool,
    // Time dilation factor
    pub dilation: f32,
    pub targets: Targets,
    pub torque_limit: ToggleValue<f32>,
    pub speed_limit: ToggleValue<f32>,
    pub position_limits: ToggleValue<(f32, f32)>,
    pub perturbation_torque: f32,
    pub perturbation_impulse: TriggerValue<f32>,
    pub motor_state: State,
    pub regulator_status: Status,
    pub regulator_output: Vector,
}

impl Gui {
    pub fn new() -> Self {
        Gui {
            reset_requested: false,
            paused: false,
            dilation: 1.,
            torque_limit: ToggleValue {
                value: 0.,
                enabled: false,
            },
            speed_limit: ToggleValue {
                value: 0.,
                enabled: false,
            },
            position_limits: ToggleValue {
                value: (0., 0.),
                enabled: false,
            },
            targets: Targets {
                torque: 0.,
                velocity: 0.,
                position: 0.,
                selected: TargetType::Torque,
            },
            perturbation_torque: 0.,
            perturbation_impulse: TriggerValue {
                value: 0.,
                triggered: false,
            },
            motor_state: State {
                currents: PhaseCurrents {
                    u: 0.,
                    v: 0.,
                    w: 0.,
                },
                velocity: 0.,
                position: 0.,
                timestamp_us: 0,
            },
            regulator_status: Status {
                torque_limited: false,
                speed_limited: false,
                position_limited: false,
            },
            regulator_output: Vector::from_components(0., 0.),
        }
    }
    pub fn show(&mut self, ctx: &Context) {
        // Reset quantities representing per-frame events
        self.reset_requested = false;
        self.perturbation_impulse.triggered = false;
        // The GUI has two areas, a left control panel and a right display panel
        SidePanel::left("controls").show(ctx, |ui| {
            ui.heading("Target");
            show_targets(ui, &mut self.targets);

            ui.heading("Limits");
            show_limits(
                ui,
                &mut self.torque_limit,
                &mut self.speed_limit,
                &mut self.position_limits,
            );

            ui.heading("Perturbations");
            show_perturberances(
                ui,
                &mut self.perturbation_torque,
                &mut self.perturbation_impulse,
            );

            ui.heading("Status");
            show_status(ui, &self.motor_state, &self.regulator_status);

            ui.heading("Time Controls");
            show_time_controls(
                ui,
                &mut self.reset_requested,
                &mut self.paused,
                &mut self.dilation,
            );
        });
        CentralPanel::default().show(ctx, |ui| {
            show_motor(ui, &self.motor_state, &self.regulator_output);
        });
    }
    pub fn update(
        &mut self,
        motor_state: State,
        regulator_status: Status,
        regulator_output: Vector,
    ) {
        self.motor_state = motor_state;
        self.regulator_status = regulator_status;
        self.regulator_output = regulator_output;
    }
}

fn show_targets(ui: &mut Ui, targets: &mut Targets) {
    Grid::new("target_grid").show(ui, |ui| {
        ui.label("Torque: ");
        ui.add(
            DragValue::new(&mut targets.torque)
                .suffix(" [N m]")
                .speed(0.001),
        );
        ui.radio_value(&mut targets.selected, TargetType::Torque, "Enabled");
        ui.end_row();

        ui.label("Velocity: ");
        ui.add(
            DragValue::new(&mut targets.velocity)
                .suffix(" [rads s-1]")
                .speed(0.01),
        );
        ui.radio_value(&mut targets.selected, TargetType::Velocity, "Enabled");
        ui.end_row();

        ui.label("Position: ");
        ui.add(
            DragValue::new(&mut targets.position)
                .suffix(" [rads]")
                .speed(0.01),
        );
        ui.radio_value(&mut targets.selected, TargetType::Position, "Enabled");
        ui.end_row();
    });
}

fn show_limits(
    ui: &mut Ui,
    torque_limit: &mut ToggleValue<f32>,
    speed_limit: &mut ToggleValue<f32>,
    position_limits: &mut ToggleValue<(f32, f32)>,
) {
    Grid::new("limit_grid").show(ui, |ui| {
        ui.label("Torque: ");
        ui.add(
            DragValue::new(&mut torque_limit.value)
                .suffix(" [N m]")
                .clamp_range(0.0..=f32::MAX)
                .speed(0.001),
        );
        ui.checkbox(&mut torque_limit.enabled, "Enabled");
        ui.end_row();

        ui.label("Speed: ");
        ui.add(
            DragValue::new(&mut speed_limit.value)
                .suffix(" [rad s-1]")
                .clamp_range(0.0..=f32::MAX),
        );
        ui.checkbox(&mut speed_limit.enabled, "Enabled");
        ui.end_row();

        ui.label("Position");
        ui.horizontal(|ui| {
            ui.add(
                DragValue::new(&mut position_limits.value.0)
                    .suffix(" [rad]")
                    .clamp_range(0.0..=(2. * PI))
                    .speed(0.1),
            );
            ui.label(" to ");
            ui.add(
                DragValue::new(&mut position_limits.value.1)
                    .suffix(" [rad]")
                    .clamp_range(0.0..=(2. * PI))
                    .speed(0.1),
            );
        });
        ui.checkbox(&mut position_limits.enabled, "Enabled");
        ui.end_row();
    });
}

fn show_perturberances(
    ui: &mut Ui,
    perturbation_torque: &mut f32,
    perturbation_impulse: &mut TriggerValue<f32>,
) {
    Grid::new("perturbation_grid").show(ui, |ui| {
        ui.label("Torque: ");
        ui.add(
            DragValue::new(perturbation_torque)
                .suffix(" [N m]")
                .speed(0.001),
        );
        ui.end_row();

        ui.label("Impulse: ");
        ui.add(
            DragValue::new(&mut perturbation_impulse.value)
                .suffix(" [N m s]")
                .speed(0.001),
        );
        if ui.button("Apply").clicked() {
            perturbation_impulse.triggered = true;
        }
        ui.end_row();
    });
}

fn show_status(ui: &mut Ui, state: &State, status: &Status) {
    ui.label(format!("Time: {}", 1E-6 * state.timestamp_us as f32));
    ui.label(format!("Rotor velocity: {} [rad s-1]", state.velocity));
    ui.label(format!("Rotor position: {} [rad]", state.position));
    ui.label(format!("Torque limited: {}", status.torque_limited));
    ui.label(format!("Speed limited: {}", status.speed_limited));
    ui.label(format!("Position limited: {}", status.position_limited));
}

fn show_time_controls(ui: &mut Ui, resetting: &mut bool, paused: &mut bool, dilation: &mut f32) {
    ui.horizontal(|ui| {
        if ui.button("Reset").clicked() {
            *resetting = true;
        }

        if ui
            .button(if *paused { "Resume" } else { "Pause" })
            .clicked()
        {
            *paused = !*paused;
        }

        ui.label("Time dilation: ");
        ui.add(
            DragValue::new(dilation)
                .suffix(" [-]")
                .speed(0.001)
                .clamp_range(0.0..=1.0),
        );
    });
}

fn show_motor(ui: &mut Ui, state: &State, output: &Vector) {
    let rotor = Arrows::new(
        Values::from_values(vec![Value::new(0.0, 0.0)]),
        Values::from_values(vec![Value::new(state.position.cos(), state.position.sin())]),
    )
    .color(Color32::BLACK);

    let voltage = Arrows::new(
        Values::from_values(vec![Value::new(0.0, 0.0)]),
        Values::from_values(vec![Value::new(
            output.magnitude * (output.phase).cos(),
            output.magnitude * (output.phase).sin(),
        )]),
    )
    .color(Color32::RED);

    Plot::new("motor_plot")
        .view_aspect(1.0)
        .data_aspect(1.0)
        .center_x_axis(true)
        .center_y_axis(true)
        .allow_drag(false)
        .allow_zoom(false)
        .allow_boxed_zoom(false)
        .allow_scroll(false)
        .show_axes([false, false])
        .show_x(false)
        .show_y(false)
        .show(ui, |plot_ui| {
            plot_ui.arrows(rotor);
            plot_ui.arrows(voltage);
        });
}
