use std::time::{Duration, Instant};

use super::{Feedback, MotorSpec};

const RUNNING_MODE_STATUS: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MotorFaultCause {
    Communication,
    StaleFeedback,
    SafePosition,
    Temperature,
    ReportedFault,
    ModeStatus,
}

#[derive(Default)]
pub(super) struct ThermalHealth {
    over_temperature: bool,
    recovery_started: Option<Instant>,
}

impl ThermalHealth {
    pub(super) fn fault_detail(
        &mut self,
        temperature: f64,
        maximum: f64,
        recovery: f64,
        recovery_duration: Duration,
        received: Instant,
    ) -> Option<String> {
        if !self.over_temperature && temperature >= maximum {
            self.over_temperature = true;
            self.recovery_started = None;
        }
        if !self.over_temperature {
            return None;
        }

        if temperature <= recovery {
            let recovery_started = *self.recovery_started.get_or_insert(received);
            if received.saturating_duration_since(recovery_started) >= recovery_duration {
                self.over_temperature = false;
                self.recovery_started = None;
                return None;
            }
        } else {
            self.recovery_started = None;
        }

        Some(format!(
            "motor over-temperature latched; cool to <= {recovery:.1} C for {} ms",
            recovery_duration.as_millis()
        ))
    }
}

pub(super) fn classify_feedback(
    value: Feedback,
    spec: &MotorSpec,
    expecting_operating_feedback: bool,
    thermal_fault: Option<String>,
) -> Option<(MotorFaultCause, String)> {
    if value.fault_bits != 0 {
        Some((
            MotorFaultCause::ReportedFault,
            format!("motor reported fault bits 0x{:04x}", value.fault_bits),
        ))
    } else if let Some(detail) = thermal_fault {
        Some((MotorFaultCause::Temperature, detail))
    } else if value.position < spec.health.safe_position_min - spec.feedback_position_tolerance
        || value.position > spec.health.safe_position_max + spec.feedback_position_tolerance
    {
        Some((
            MotorFaultCause::SafePosition,
            format!(
                "feedback position {:.4} outside configured safe range [{:.4}, {:.4}] (protocol tolerance {:.6})",
                value.position,
                spec.health.safe_position_min,
                spec.health.safe_position_max,
                spec.feedback_position_tolerance
            ),
        ))
    } else if expecting_operating_feedback && value.mode_status != RUNNING_MODE_STATUS {
        Some((
            MotorFaultCause::ModeStatus,
            format!(
                "motor reported mode status {} while operating status {} was expected",
                value.mode_status, RUNNING_MODE_STATUS
            ),
        ))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ResolvedTopology;

    fn motor_spec() -> MotorSpec {
        motor_spec_named("left_hip_pitch")
    }

    fn motor_spec_named(name: &str) -> MotorSpec {
        ResolvedTopology::load(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bello.yaml"))
            .unwrap()
            .motors()
            .find(|(_, motor)| motor.spec.name.as_str() == name)
            .unwrap()
            .1
            .spec
            .clone()
    }

    #[test]
    fn thermal_fault_requires_hysteresis_and_continuous_cooldown() {
        let mut health = ThermalHealth::default();
        let started = Instant::now();
        let cooldown = Duration::from_secs(2);

        assert!(
            health
                .fault_detail(80.0, 80.0, 75.0, cooldown, started)
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(100),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    76.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(1_500),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(1_600),
                )
                .is_some()
        );
        assert!(
            health
                .fault_detail(
                    74.0,
                    80.0,
                    75.0,
                    cooldown,
                    started + Duration::from_millis(3_600),
                )
                .is_none()
        );
    }

    #[test]
    fn reported_fault_bits_take_precedence() {
        let fault = classify_feedback(
            Feedback {
                fault_bits: 1,
                mode_status: RUNNING_MODE_STATUS,
                ..Default::default()
            },
            &motor_spec(),
            true,
            None,
        );
        assert!(matches!(fault, Some((MotorFaultCause::ReportedFault, _))));
    }

    #[test]
    fn wrong_mode_faults_only_while_operating_feedback_is_expected() {
        let value = Feedback {
            mode_status: 1,
            ..Default::default()
        };
        let spec = motor_spec();
        assert!(classify_feedback(value, &spec, false, None).is_none());
        assert!(matches!(
            classify_feedback(value, &spec, true, None),
            Some((MotorFaultCause::ModeStatus, _))
        ));
    }

    #[test]
    fn feedback_at_a_urdf_boundary_allows_protocol_quantization() {
        let spec = motor_spec_named("left_knee_mix_1");
        let within_quantization = Feedback {
            position: -0.5 * spec.feedback_position_tolerance,
            ..Default::default()
        };
        assert!(classify_feedback(within_quantization, &spec, false, None).is_none());

        let outside_quantization = Feedback {
            position: -2.0 * spec.feedback_position_tolerance,
            ..Default::default()
        };
        assert!(matches!(
            classify_feedback(outside_quantization, &spec, false, None),
            Some((MotorFaultCause::SafePosition, _))
        ));
    }
}
