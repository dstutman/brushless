#![no_std]
pub mod modulation;
pub mod pid;
pub mod regulator;
mod util;

pub use util::Vector;

#[cfg(test)]
mod tests {

    use crate::regulator::{MotorProperties, PhaseCurrents, RegulatorConfig, State};

    fn init_logs() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(log::LevelFilter::Debug)
            .try_init()
            .unwrap();
    }

    #[test]
    fn single_sample() {
        init_logs();

        let mut regulator = RegulatorConfig::new(MotorProperties {
            inertia: 1E-5,
            pole_pairs: 7,
            phase_resistance: 22.,
            torque_constant: 5E-2,
        })
        .realize();

        let state = State {
            currents: PhaseCurrents {
                u: 1.,
                v: 1.,
                w: -2.,
            },
            position: -1.5,
            velocity: 0.,
            timestamp_us: 1000,
        };
        regulator.update(state);

        let duties = crate::modulation::space_vector_modulation(regulator.output());

        log::debug!("Calculated phase duties: {:?}", duties)
    }
}
