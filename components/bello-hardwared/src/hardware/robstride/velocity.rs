use std::f64::consts::TAU;
use std::time::{Duration, Instant};

pub(super) struct MotorVelocityFilter {
    cutoff_hz: f64,
    maximum_sample_interval: Duration,
    previous: Option<(f64, Instant)>,
}

impl MotorVelocityFilter {
    pub(super) fn new(cutoff_hz: f64, maximum_sample_interval: Duration) -> Self {
        Self {
            cutoff_hz,
            maximum_sample_interval,
            previous: None,
        }
    }

    pub(super) fn apply(&mut self, velocity: f64, received: Instant) -> f64 {
        let filtered = self
            .previous
            .and_then(|(previous_filtered, previous_received)| {
                received
                    .checked_duration_since(previous_received)
                    .filter(|interval| {
                        !interval.is_zero() && *interval <= self.maximum_sample_interval
                    })
                    .map(|interval| {
                        let alpha = 1.0 - (-TAU * self.cutoff_hz * interval.as_secs_f64()).exp();
                        previous_filtered + alpha * (velocity - previous_filtered)
                    })
            })
            .unwrap_or(velocity);
        self.previous = Some((filtered, received));
        filtered
    }

    pub(super) fn reset(&mut self) {
        self.previous = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filter(cutoff_hz: f64) -> MotorVelocityFilter {
        MotorVelocityFilter::new(cutoff_hz, Duration::from_millis(50))
    }

    #[test]
    fn first_sample_passes_through() {
        assert_eq!(filter(15.0).apply(2.0, Instant::now()), 2.0);
    }

    #[test]
    fn coefficient_uses_cutoff_and_sample_interval() {
        let mut filter = filter(15.0);
        let start = Instant::now();

        filter.apply(2.0, start);
        let filtered = filter.apply(-2.0, start + Duration::from_millis(10));
        let alpha = 1.0 - (-TAU * 15.0 * 0.01).exp();
        assert!((filtered - (2.0 - 4.0 * alpha)).abs() < 1.0e-12);
    }

    #[test]
    fn recursive_state_uses_the_previous_filtered_value() {
        let mut filter = filter(15.0);
        let start = Instant::now();
        let interval = Duration::from_millis(10);
        let alpha = 1.0 - (-TAU * 15.0 * interval.as_secs_f64()).exp();

        filter.apply(2.0, start);
        let second = filter.apply(-2.0, start + interval);
        let third = filter.apply(-2.0, start + 2 * interval);
        assert!((third - (second + alpha * (-2.0 - second))).abs() < 1.0e-12);
    }

    #[test]
    fn higher_cutoff_tracks_a_step_more_quickly() {
        let start = Instant::now();
        let received = start + Duration::from_millis(10);
        let mut slower = filter(10.0);
        let mut faster = filter(30.0);

        slower.apply(0.0, start);
        faster.apply(0.0, start);
        assert!(faster.apply(1.0, received) > slower.apply(1.0, received));
    }

    #[test]
    fn feedback_gap_restarts_the_filter() {
        let mut filter = filter(15.0);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(100)), -2.0);
    }

    #[test]
    fn non_increasing_timestamp_restarts_the_filter() {
        let mut filter = filter(15.0);
        let start = Instant::now();

        filter.apply(2.0, start);
        assert_eq!(filter.apply(-2.0, start), -2.0);
        assert!(filter.apply(4.0, start + Duration::from_millis(10)) > -2.0);
    }

    #[test]
    fn reset_discards_the_previous_sample() {
        let mut filter = filter(15.0);
        let start = Instant::now();

        filter.apply(2.0, start);
        filter.reset();
        assert_eq!(filter.apply(-2.0, start + Duration::from_millis(10)), -2.0);
    }
}
