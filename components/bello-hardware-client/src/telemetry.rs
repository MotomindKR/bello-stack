use bello_hardware_api::watch_telemetry_response::Payload;
use bello_hardware_api::{WatchTelemetryRequest, WatchTelemetryResponse};
use tonic::Streaming;

use crate::client::require_nonempty;
use crate::{Error, HardwareClient};

/// Selection applied to the event-based telemetry stream.
///
/// `Default` selects group events for every group and no optional event class.
/// Use [`Self::all`] when a complete diagnostic subscription is intended.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TelemetryFilter {
    groups: Vec<String>,
    include_motors: bool,
    include_imu: bool,
    include_health: bool,
    imus: Vec<String>,
    include_command_policy_events: bool,
}

impl TelemetryFilter {
    /// Selects every group, motor, IMU, health, and command-policy event.
    pub fn all() -> Self {
        Self {
            groups: Vec::new(),
            include_motors: true,
            include_imu: true,
            include_health: true,
            imus: Vec::new(),
            include_command_policy_events: true,
        }
    }

    /// Restricts group, motor, and command-policy events. Empty means all.
    pub fn selected_groups(mut self, groups: impl IntoIterator<Item = impl Into<String>>) -> Self {
        self.groups = groups.into_iter().map(Into::into).collect();
        self
    }

    pub fn include_motors(mut self, include: bool) -> Self {
        self.include_motors = include;
        self
    }

    pub fn include_imu(mut self, include: bool) -> Self {
        self.include_imu = include;
        self
    }

    /// Restricts IMU events. Empty means all when IMU telemetry is enabled.
    pub fn selected_imus(mut self, imus: impl IntoIterator<Item = impl Into<String>>) -> Self {
        self.imus = imus.into_iter().map(Into::into).collect();
        self
    }

    pub fn include_health(mut self, include: bool) -> Self {
        self.include_health = include;
        self
    }

    pub fn include_command_policy_events(mut self, include: bool) -> Self {
        self.include_command_policy_events = include;
        self
    }

    fn validate(&self) -> Result<(), Error> {
        for group in &self.groups {
            require_nonempty(group, "telemetry group selector")?;
        }
        if !self.include_imu && !self.imus.is_empty() {
            return Err(Error::InvalidArgument(
                "IMU selectors require IMU telemetry to be enabled".into(),
            ));
        }
        for imu in &self.imus {
            require_nonempty(imu, "telemetry IMU selector")?;
        }
        Ok(())
    }

    pub fn into_request(self) -> WatchTelemetryRequest {
        WatchTelemetryRequest {
            groups: self.groups,
            include_motors: self.include_motors,
            include_imu: self.include_imu,
            include_health: self.include_health,
            imus: self.imus,
            include_command_policy_events: self.include_command_policy_events,
        }
    }
}

/// One telemetry event with the daemon-relative source timestamp preserved.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetryEvent {
    pub monotonic_ns: u64,
    pub payload: Payload,
}

pub struct TelemetryStream {
    inner: Streaming<WatchTelemetryResponse>,
}

impl TelemetryStream {
    pub(crate) async fn open(
        client: &HardwareClient,
        filter: TelemetryFilter,
    ) -> Result<Self, Error> {
        filter.validate()?;
        let mut grpc = client.inner.clone();
        let inner = grpc
            .watch_telemetry(filter.into_request())
            .await?
            .into_inner();
        Ok(Self { inner })
    }

    /// Receives the next event. `None` means the server ended the stream.
    pub async fn message(&mut self) -> Result<Option<TelemetryEvent>, Error> {
        let Some(response) = self.inner.message().await? else {
            return Ok(None);
        };
        let payload = response
            .payload
            .ok_or_else(|| Error::Protocol("telemetry response has no payload".into()))?;
        Ok(Some(TelemetryEvent {
            monotonic_ns: response.monotonic_ns,
            payload,
        }))
    }

    pub fn into_inner(self) -> Streaming<WatchTelemetryResponse> {
        self.inner
    }
}

impl std::fmt::Debug for TelemetryStream {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("TelemetryStream { .. }")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_requests_every_event_class() {
        let request = TelemetryFilter::all().into_request();
        assert!(request.groups.is_empty());
        assert!(request.imus.is_empty());
        assert!(request.include_motors);
        assert!(request.include_imu);
        assert!(request.include_health);
        assert!(request.include_command_policy_events);
    }

    #[test]
    fn imu_selectors_require_imu_events() {
        let filter = TelemetryFilter::default().selected_imus(["body"]);
        assert!(filter.validate().is_err());
    }
}
