use std::collections::BTreeMap;
use std::time::Duration;

use bello_robot_description::CoordinateState;

#[derive(Clone, Copy, Debug)]
struct TimedCoordinate {
    source_ns: u64,
    position: f64,
    velocity: f64,
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

    fn sample_at(
        &self,
        target_ns: u64,
        maximum_offset: Duration,
    ) -> Option<(CoordinateState, u64)> {
        let latest = self.latest?;
        let maximum_ns = duration_ns(maximum_offset);
        if let Some(previous) = self.previous
            && previous.source_ns <= target_ns
            && target_ns <= latest.source_ns
        {
            let span = latest.source_ns - previous.source_ns;
            if span == 0 {
                return Some((
                    CoordinateState {
                        position: latest.position,
                        velocity: latest.velocity,
                    },
                    0,
                ));
            }
            let alpha = (target_ns - previous.source_ns) as f64 / span as f64;
            let offset = (target_ns - previous.source_ns).min(latest.source_ns - target_ns);
            if offset > maximum_ns {
                return None;
            }
            return Some((
                CoordinateState {
                    position: previous.position + alpha * (latest.position - previous.position),
                    velocity: previous.velocity + alpha * (latest.velocity - previous.velocity),
                },
                offset,
            ));
        }

        let reference = if target_ns < latest.source_ns {
            self.previous.unwrap_or(latest)
        } else {
            latest
        };
        let offset = target_ns.abs_diff(reference.source_ns);
        if offset > maximum_ns {
            return None;
        }
        let dt = (target_ns as i128 - reference.source_ns as i128) as f64 * 1.0e-9;
        Some((
            CoordinateState {
                position: reference.position + reference.velocity * dt,
                velocity: reference.velocity,
            },
            offset,
        ))
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
    pub maximum_offset: Duration,
}

impl CoordinateHistory {
    pub fn clear(&mut self) {
        self.timelines.clear();
    }

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
        let mut maximum_observed_offset = 0;
        for name in names {
            let timeline = self.timelines.get(name)?;
            let latest = timeline.latest?;
            let (value, offset) = timeline.sample_at(target_ns, maximum_offset)?;
            values.insert(name.clone(), value);
            epoch_ns = epoch_ns.min(latest.source_ns);
            maximum_observed_offset = maximum_observed_offset.max(offset);
        }
        Some(AlignedCoordinates {
            values,
            epoch_ns,
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
            .sample_at(&["joint".into()], 150_000_000, Duration::from_millis(60))
            .unwrap();
        let joint = &sample.values["joint"];
        assert!((joint.position - 0.1).abs() < 1.0e-12);
        assert!((joint.velocity - 2.0).abs() < 1.0e-12);
        assert_eq!(sample.maximum_offset, Duration::from_millis(50));
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
