use brushless::{
    modulation::PhaseDuties,
    regulator::{MotorProperties, PhaseCurrents},
    Vector,
};
use rapier2d::prelude::*;

pub struct RotorSimulator {
    motor: MotorProperties,
    supply_voltage: f32,
    phase_duties: PhaseDuties,
    phase_currents: PhaseCurrents,
    perturbation_torque: Real,
    perturbation_impulse: Option<Real>,
    physics_pipeline: PhysicsPipeline,
    rigid_bodies: RigidBodySet,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rotor_handle: RigidBodyHandle,
}

impl RotorSimulator {
    pub fn new(motor: MotorProperties, supply_voltage: f32) -> Self {
        let mut rigid_bodies = RigidBodySet::new();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = BroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let colliders = ColliderSet::new();
        let impulse_joints = ImpulseJointSet::new();
        let multibody_joints = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();

        let rotor = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 0.0])
            .additional_mass_properties(MassProperties::new(point![0.0, 0.0], 1.0, motor.inertia))
            .build();

        let rotor_handle = rigid_bodies.insert(rotor);

        RotorSimulator {
            motor,
            phase_currents: PhaseCurrents::default(),
            supply_voltage,
            phase_duties: PhaseDuties {
                u: 0.,
                v: 0.,
                w: 0.,
            },
            perturbation_torque: 0.0,
            perturbation_impulse: None,
            physics_pipeline,
            rigid_bodies,
            island_manager,
            broad_phase,
            narrow_phase,
            colliders,
            impulse_joints,
            multibody_joints,
            ccd_solver,
            rotor_handle,
        }
    }

    pub fn step(&mut self, dt: Real) {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.set_inv_dt(1.0 / dt);

        // Rotor governing laws
        let rotor = &mut self.rigid_bodies[self.rotor_handle];
        // Apply perturbation impulse
        let impulse = match self.perturbation_impulse {
            Some(val) => {
                self.perturbation_impulse = None;
                val
            }
            None => 0.0,
        };
        rotor.apply_torque_impulse(impulse, true);

        // Recalculate torques
        rotor.reset_torques(true);

        // Assume that the current instantaneously follows the voltage.
        self.phase_currents = PhaseCurrents {
            u: self.phase_duties.u * self.supply_voltage / self.motor.phase_resistance,
            v: self.phase_duties.v * self.supply_voltage / self.motor.phase_resistance,
            w: self.phase_duties.w * self.supply_voltage / self.motor.phase_resistance,
        };
        let current_vector = Vector::from_components(
            self.phase_currents.u,
            1. / 3_f32.sqrt() * (self.phase_currents.v - self.phase_currents.w),
        );
        let torque = current_vector.magnitude
            * self.motor.torque_constant
            * (current_vector.phase - rotor.rotation().angle()).sin();
        rotor.add_torque(torque, true);

        // Add some friction
        rotor.add_torque(-rotor.angvel() * 1E-5, true);

        rotor.add_torque(self.perturbation_torque, true);

        self.physics_pipeline.step(
            &vector![0.0, 0.0],
            &integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        )
    }

    pub fn set_phase_duties(&mut self, phase_duties: PhaseDuties) {
        self.phase_duties = phase_duties;
    }

    pub fn set_perturbation_torque(&mut self, torque: Real) {
        self.perturbation_torque = torque;
    }

    pub fn apply_impulse(&mut self, impulse: Real) {
        self.perturbation_impulse = Some(impulse);
    }

    pub fn phase_currents(&self) -> PhaseCurrents {
        self.phase_currents
    }

    pub fn angle(&self) -> Real {
        self.rigid_bodies[self.rotor_handle].rotation().angle()
    }

    pub fn angular_velocity(&self) -> Real {
        self.rigid_bodies[self.rotor_handle].angvel()
    }

    pub fn reset(&mut self) {
        let rotor = &mut self.rigid_bodies[self.rotor_handle];

        // Reset all perturbations
        self.perturbation_impulse = None;
        self.perturbation_torque = 0.0;
        self.phase_duties = PhaseDuties::default();

        // Reset physical state
        rotor.set_rotation(0.0, true);
        rotor.set_angvel(0.0, true);
    }
}
