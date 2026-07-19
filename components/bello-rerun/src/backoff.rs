use std::time::Duration;

#[derive(Clone, Debug)]
pub(crate) struct Backoff {
    initial: Duration,
    maximum: Duration,
    next: Duration,
}

impl Backoff {
    pub(crate) fn new(initial: Duration, maximum: Duration) -> Self {
        debug_assert!(!initial.is_zero());
        debug_assert!(maximum >= initial);
        Self {
            initial,
            maximum,
            next: initial,
        }
    }

    pub(crate) fn next_delay(&mut self) -> Duration {
        let delay = self.next;
        self.next = self.next.saturating_mul(2).min(self.maximum);
        delay
    }

    pub(crate) fn reset(&mut self) {
        self.next = self.initial;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delay_doubles_until_the_maximum_and_resets() {
        let mut backoff = Backoff::new(Duration::from_millis(100), Duration::from_millis(450));
        assert_eq!(backoff.next_delay(), Duration::from_millis(100));
        assert_eq!(backoff.next_delay(), Duration::from_millis(200));
        assert_eq!(backoff.next_delay(), Duration::from_millis(400));
        assert_eq!(backoff.next_delay(), Duration::from_millis(450));
        assert_eq!(backoff.next_delay(), Duration::from_millis(450));
        backoff.reset();
        assert_eq!(backoff.next_delay(), Duration::from_millis(100));
    }
}
