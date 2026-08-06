use std::time::{Duration, Instant};

pub(super) struct MotorVelocityFilter {
    current_sample_weight: f64,
    maximum_sample_interval: Duration,
    previous: Option<(f64, Instant)>,
}

impl MotorVelocityFilter {
    pub(super) fn new(current_sample_weight: f64, maximum_sample_interval: Duration) -> Self {
        Self {
            current_sample_weight,
            maximum_sample_interval,
            previous: None,
        }
    }

    pub(super) fn apply(&mut self, velocity: f64, received: Instant) -> f64 {
        let filtered = self
            .previous
            .and_then(|(previous_velocity, previous_received)| {
                received
                    .checked_duration_since(previous_received)
                    .filter(|interval| {
                        !interval.is_zero() && *interval <= self.maximum_sample_interval
                    })
                    .map(|_| {
                        self.current_sample_weight * velocity
                            + (1.0 - self.current_sample_weight) * previous_velocity
                    })
            })
            .unwrap_or(velocity);
        self.previous = Some((velocity, received));
        filtered
    }

    pub(super) fn reset(&mut self) {
        self.previous = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filter(weight: f64) -> MotorVelocityFilter {
        MotorVelocityFilter::new(weight, Duration::from_millis(50))
    }

    #[test]
    fn first_sample_passes_through() {
        assert_eq!(filter(0.5).apply(2.0, Instant::now()), 2.0);
    }

    #[test]
    fn default_weights_cancel_alternating_velocity() {
        let mut filter = filter(0.5);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(10)), 0.0);
    }

    #[test]
    fn current_sample_weight_is_configurable() {
        let mut filter = filter(0.75);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(10)), -1.0);
    }

    #[test]
    fn feedback_gap_restarts_the_filter() {
        let mut filter = filter(0.5);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(100)), -2.0);
    }

    #[test]
    fn non_increasing_timestamp_restarts_the_filter() {
        let mut filter = filter(0.5);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start), -2.0);
        assert_eq!(filter.apply(4.0, start + Duration::from_millis(10)), 1.0);
    }

    #[test]
    fn reset_discards_the_previous_sample() {
        let mut filter = filter(0.5);
        let start = Instant::now();

        filter.apply(2.0, start);
        filter.reset();
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(10)), -2.0);
    }
}
