use std::collections::VecDeque;
use std::time::{Duration, Instant};

use robstride_protocol::Feedback;

const WINDOW: Duration = Duration::from_millis(100);
const MIN_SAMPLES: usize = 5;
const POSITION_SPAN: f64 = 0.003;
const REPORTED_SPEED: f64 = 0.15;

// Use the vendor setup tool's position span and median velocity thresholds.
// Unwrap successive positions so encoder rollover does not appear as motion.
pub(super) struct StationaryCheck {
    span: f64,
    previous_raw: Option<f64>,
    position: f64,
    samples: VecDeque<(Instant, f64, f64)>,
}

impl StationaryCheck {
    pub(super) fn new(span: f64) -> Self {
        Self {
            span,
            previous_raw: None,
            position: 0.0,
            samples: VecDeque::new(),
        }
    }

    pub(super) fn clear(&mut self) {
        self.previous_raw = None;
        self.position = 0.0;
        self.samples.clear();
    }

    pub(super) fn observe(&mut self, feedback: Feedback, now: Instant) -> bool {
        if feedback.mode_status != 0 {
            self.clear();
            return false;
        }
        if self
            .samples
            .back()
            .is_some_and(|&(previous, _, _)| now.duration_since(previous) > WINDOW)
        {
            self.clear();
        }
        if let Some(previous) = self.previous_raw {
            let half_span = self.span / 2.0;
            self.position +=
                (feedback.position - previous + half_span).rem_euclid(self.span) - half_span;
        }
        self.previous_raw = Some(feedback.position);
        while self.samples.len() > 1 && now.duration_since(self.samples[1].0) >= WINDOW {
            self.samples.pop_front();
        }
        self.samples
            .push_back((now, self.position, feedback.velocity));

        if self.samples.len() < MIN_SAMPLES || now.duration_since(self.samples[0].0) < WINDOW {
            return false;
        }
        let (low, high) = self.samples.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(low, high), &(_, position, _)| (low.min(position), high.max(position)),
        );
        let mut speeds: Vec<_> = self.samples.iter().map(|sample| sample.2).collect();
        speeds.sort_unstable_by(f64::total_cmp);
        let middle = speeds.len() / 2;
        let median = (speeds[(speeds.len() - 1) / 2] + speeds[middle]) / 2.0;
        high - low <= POSITION_SPAN && median.abs() < REPORTED_SPEED
    }
}
