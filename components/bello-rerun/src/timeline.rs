#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TimelineReset {
    pub(crate) previous_source_ns: u64,
    pub(crate) new_source_ns: u64,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct Timeline {
    offset_ns: u64,
    last_source_ns: Option<u64>,
    last_output_ns: Option<u64>,
    restart_eligible: bool,
}

impl Timeline {
    pub(crate) fn begin_connection(&mut self) {
        self.restart_eligible = true;
    }

    pub(crate) fn map(&mut self, source_ns: u64) -> (u64, Option<TimelineReset>) {
        let source_reversed = self.last_source_ns.filter(|previous| source_ns < *previous);
        let reset = source_reversed
            .filter(|_| self.restart_eligible)
            .map(|previous_source_ns| TimelineReset {
                previous_source_ns,
                new_source_ns: source_ns,
            });

        let candidate = source_ns.saturating_add(self.offset_ns);
        let mapped = match self.last_output_ns {
            Some(previous_output) if reset.is_some() => {
                let next_output = previous_output.saturating_add(1);
                self.offset_ns = next_output.saturating_sub(source_ns);
                source_ns.saturating_add(self.offset_ns)
            }
            Some(previous_output) if candidate < previous_output => previous_output,
            _ => candidate,
        };

        self.last_source_ns = Some(if reset.is_some() {
            source_ns
        } else {
            self.last_source_ns
                .map_or(source_ns, |last| last.max(source_ns))
        });
        self.last_output_ns = Some(mapped);
        self.restart_eligible = false;
        (mapped, reset)
    }

    pub(crate) fn marker_time(&mut self) -> u64 {
        let timestamp = self
            .last_output_ns
            .map_or(0, |previous| previous.saturating_add(1));
        self.last_output_ns = Some(timestamp);
        timestamp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_timestamps_are_preserved_without_a_reset() {
        let mut timeline = Timeline::default();
        timeline.begin_connection();
        assert_eq!(timeline.map(10), (10, None));
        assert_eq!(timeline.map(20), (20, None));
    }

    #[test]
    fn daemon_restart_is_rebased_after_the_previous_output() {
        let mut timeline = Timeline::default();
        timeline.begin_connection();
        assert_eq!(timeline.map(100), (100, None));
        let marker = timeline.marker_time();
        assert_eq!(marker, 101);

        timeline.begin_connection();
        let (mapped, reset) = timeline.map(5);
        assert_eq!(mapped, 102);
        assert_eq!(
            reset,
            Some(TimelineReset {
                previous_source_ns: 100,
                new_source_ns: 5,
            })
        );
        assert_eq!(timeline.map(15), (112, None));
    }

    #[test]
    fn reconnect_markers_cannot_make_the_timeline_go_backwards() {
        let mut timeline = Timeline::default();
        timeline.begin_connection();
        assert_eq!(timeline.map(10), (10, None));
        assert_eq!(timeline.marker_time(), 11);
        assert_eq!(timeline.marker_time(), 12);
        timeline.begin_connection();
        assert_eq!(timeline.map(11), (12, None));
    }

    #[test]
    fn in_stream_reordering_is_clamped_without_marking_a_restart() {
        let mut timeline = Timeline::default();
        timeline.begin_connection();
        assert_eq!(timeline.map(100), (100, None));
        assert_eq!(timeline.map(90), (100, None));
        assert_eq!(timeline.map(110), (110, None));
    }
}
