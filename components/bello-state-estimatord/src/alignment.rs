use std::collections::BTreeMap;
use std::time::Duration;

use bello_robot_description::CoordinateState;

#[derive(Clone, Copy, Debug)]
struct TimedCoordinate {
    source_ns: u64,
    position: f64,
    velocity: f64,
}

struct AlignedCoordinate {
    value: CoordinateState,
    oldest_source_ns: u64,
    maximum_offset_ns: u64,
}

#[derive(Clone, Copy, Debug, Default)]
struct CoordinateTimeline {
    previous: Option<TimedCoordinate>,
    latest: Option<TimedCoordinate>,
}

impl CoordinateTimeline {
    fn observe(&mut self, sample: TimedCoordinate) {
        let Some(latest) = self.latest else {
            self.latest = Some(sample);
            return;
        };
        if sample.source_ns > latest.source_ns {
            self.previous = Some(latest);
            self.latest = Some(sample);
        } else if sample.source_ns == latest.source_ns {
            self.latest = Some(sample);
        }
    }

    fn sample_at(&self, target_ns: u64, maximum_ns: u64) -> Option<AlignedCoordinate> {
        let latest = self.latest?;
        let mut reference = latest;
        let mut offset = target_ns.abs_diff(latest.source_ns);
        if let Some(previous) = self.previous
            && target_ns < latest.source_ns
        {
            let previous_offset = target_ns.abs_diff(previous.source_ns);
            let maximum_offset_ns = previous_offset.max(offset);
            if previous.source_ns < target_ns && maximum_offset_ns <= maximum_ns {
                let span = latest.source_ns - previous.source_ns;
                let alpha = previous_offset as f64 / span as f64;
                return Some(AlignedCoordinate {
                    value: CoordinateState {
                        position: previous.position + alpha * (latest.position - previous.position),
                        velocity: previous.velocity + alpha * (latest.velocity - previous.velocity),
                    },
                    oldest_source_ns: previous.source_ns,
                    maximum_offset_ns,
                });
            }
            if previous_offset <= offset {
                reference = previous;
                offset = previous_offset;
            }
        }

        if offset > maximum_ns {
            return None;
        }
        let dt = (target_ns as i128 - reference.source_ns as i128) as f64 * 1.0e-9;
        Some(AlignedCoordinate {
            value: CoordinateState {
                position: reference.position + reference.velocity * dt,
                velocity: reference.velocity,
            },
            oldest_source_ns: reference.source_ns,
            maximum_offset_ns: offset,
        })
    }
}

#[derive(Clone, Debug, Default)]
pub struct CoordinateHistory {
    timelines: BTreeMap<String, CoordinateTimeline>,
}

#[derive(Clone, Debug)]
pub struct AlignedCoordinates {
    pub values: BTreeMap<String, CoordinateState>,
    pub epoch_ns: u64,
    pub oldest_source_ns: u64,
    pub maximum_offset: Duration,
}

impl CoordinateHistory {
    pub fn observe(
        &mut self,
        coordinate: &str,
        monotonic_ns: u64,
        feedback_age: Duration,
        position: f64,
        velocity: f64,
    ) {
        if !position.is_finite() || !velocity.is_finite() {
            return;
        }
        let sample = TimedCoordinate {
            source_ns: monotonic_ns.saturating_sub(duration_ns(feedback_age)),
            position,
            velocity,
        };
        self.timelines
            .entry(coordinate.to_owned())
            .or_default()
            .observe(sample);
    }

    pub fn sample_at(
        &self,
        names: &[String],
        target_ns: u64,
        maximum_offset: Duration,
    ) -> Option<AlignedCoordinates> {
        let mut values = BTreeMap::new();
        let mut epoch_ns = u64::MAX;
        let mut oldest_source_ns = u64::MAX;
        let mut maximum_observed_offset = 0;
        let maximum_ns = duration_ns(maximum_offset);
        for name in names {
            let timeline = self.timelines.get(name)?;
            let latest = timeline.latest?;
            let sample = timeline.sample_at(target_ns, maximum_ns)?;
            values.insert(name.clone(), sample.value);
            epoch_ns = epoch_ns.min(latest.source_ns);
            oldest_source_ns = oldest_source_ns.min(sample.oldest_source_ns);
            maximum_observed_offset = maximum_observed_offset.max(sample.maximum_offset_ns);
        }
        Some(AlignedCoordinates {
            values,
            epoch_ns,
            oldest_source_ns,
            maximum_offset: Duration::from_nanos(maximum_observed_offset),
        })
    }
}

pub fn source_time_ns(monotonic_ns: u64, age: Duration) -> u64 {
    monotonic_ns.saturating_sub(duration_ns(age))
}

fn duration_ns(duration: Duration) -> u64 {
    duration.as_nanos().min(u64::MAX as u128) as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_coordinates_to_the_imu_source_time() {
        let mut history = CoordinateHistory::default();
        history.observe("joint", 100_000_000, Duration::ZERO, 0.0, 1.0);
        history.observe("joint", 200_000_000, Duration::ZERO, 0.2, 3.0);

        let sample = history
            .sample_at(&["joint".into()], 140_000_000, Duration::from_millis(60))
            .unwrap();
        let joint = &sample.values["joint"];
        assert!((joint.position - 0.08).abs() < 1.0e-12);
        assert!((joint.velocity - 1.8).abs() < 1.0e-12);
        assert_eq!(sample.oldest_source_ns, 100_000_000);
        assert_eq!(sample.maximum_offset, Duration::from_millis(60));
    }

    #[test]
    fn extrapolates_from_the_only_fresh_bracket_endpoint() {
        let mut history = CoordinateHistory::default();
        history.observe("joint", 100_000_000, Duration::ZERO, 0.4, 2.0);
        history.observe("joint", 220_000_000, Duration::ZERO, 1.0, 3.0);

        let names = ["joint".into()];
        let limit = Duration::from_millis(50);
        for (target, source, position, velocity) in [
            (110_000_000_u64, 100_000_000, 0.42, 2.0),
            (210_000_000, 220_000_000, 0.97, 3.0),
            (220_000_000, 220_000_000, 1.0, 3.0),
        ] {
            let sample = history.sample_at(&names, target, limit).unwrap();
            let joint = &sample.values["joint"];
            assert!((joint.position - position).abs() < 1.0e-12, "{joint:?}");
            assert_eq!(joint.velocity, velocity);
            assert_eq!(sample.oldest_source_ns, source);
            assert_eq!(
                sample.maximum_offset,
                Duration::from_nanos(target.abs_diff(source))
            );
        }
        assert!(history.sample_at(&names, 160_000_000, limit).is_none());
    }

    #[test]
    fn velocity_extrapolation_is_bounded_by_freshness() {
        let mut history = CoordinateHistory::default();
        history.observe("joint", 100_000_000, Duration::ZERO, 0.4, 2.0);

        let sample = history
            .sample_at(&["joint".into()], 110_000_000, Duration::from_millis(20))
            .unwrap();
        assert!((sample.values["joint"].position - 0.42).abs() < 1.0e-12);
        assert!(
            history
                .sample_at(&["joint".into()], 130_000_000, Duration::from_millis(20))
                .is_none()
        );
    }

    #[test]
    fn feedback_age_is_removed_from_the_event_timestamp() {
        let mut history = CoordinateHistory::default();
        history.observe("joint", 110_000_000, Duration::from_millis(10), 0.4, 0.0);
        let sample = history
            .sample_at(&["joint".into()], 100_000_000, Duration::ZERO)
            .unwrap();
        assert_eq!(sample.epoch_ns, 100_000_000);
    }
}
