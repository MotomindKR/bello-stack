use std::fmt;
use std::time::{Duration, Instant};

use bello_hardware_api::hardware_service_client::HardwareServiceClient;
use bello_hardware_api::{AcquireLeaseRequest, ListHardwareRequest, ListHardwareResponse};
use tonic::transport::{Channel, Endpoint};

use crate::lease::GroupLease;
use crate::telemetry::{TelemetryFilter, TelemetryStream};
use crate::{ConnectOptions, Error, LeaseDuration};

#[derive(Clone)]
pub struct HardwareClient {
    pub(crate) inner: HardwareServiceClient<Channel>,
}

impl fmt::Debug for HardwareClient {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("HardwareClient { .. }")
    }
}

impl HardwareClient {
    /// Connects with native certificate roots for HTTPS and the default
    /// timeout/keepalive policy from [`ConnectOptions`].
    pub async fn connect(endpoint: impl Into<String>) -> Result<Self, Error> {
        Self::connect_with(ConnectOptions::new(endpoint)).await
    }

    pub async fn connect_with(options: ConnectOptions) -> Result<Self, Error> {
        Self::connect_endpoint(options.into_endpoint()?).await
    }

    pub async fn connect_endpoint(endpoint: Endpoint) -> Result<Self, Error> {
        Ok(Self::from_channel(endpoint.connect().await?))
    }

    pub fn from_channel(channel: Channel) -> Self {
        Self {
            inner: HardwareServiceClient::new(channel),
        }
    }

    /// Returns a generated client sharing this handle's channel.
    pub fn grpc_client(&self) -> HardwareServiceClient<Channel> {
        self.inner.clone()
    }

    pub async fn inventory(&self) -> Result<ListHardwareResponse, Error> {
        let mut client = self.inner.clone();
        Ok(client
            .list_hardware(ListHardwareRequest {})
            .await?
            .into_inner())
    }

    pub async fn watch(&self, filter: TelemetryFilter) -> Result<TelemetryStream, Error> {
        TelemetryStream::open(self, filter).await
    }

    /// Renew before [`GroupLease::deadline`], including during long lifecycle calls.
    pub async fn acquire_lease(
        &self,
        group: impl Into<String>,
        client_name: impl Into<String>,
        duration: LeaseDuration,
    ) -> Result<GroupLease, Error> {
        let group = group.into();
        let client_name = client_name.into();
        require_nonempty(&group, "group")?;
        require_nonempty(&client_name, "client_name")?;
        let requested_duration_ms = duration.to_milliseconds()?;
        let request_started = Instant::now();
        let mut client = self.inner.clone();
        let response = client
            .acquire_lease(AcquireLeaseRequest {
                group: group.clone(),
                client_name,
                requested_duration_ms,
            })
            .await?
            .into_inner();
        GroupLease::from_acquired(self.clone(), group, request_started, response)
    }
}

pub(crate) fn conservative_deadline(
    request_started: Instant,
    expires_in_ms: u64,
) -> Result<Instant, Error> {
    if expires_in_ms == 0 {
        return Err(Error::Protocol(
            "lease response contains a zero remaining duration".into(),
        ));
    }
    request_started
        .checked_add(Duration::from_millis(expires_in_ms))
        .ok_or_else(|| Error::Protocol("lease deadline exceeds local Instant range".into()))
}

pub(crate) fn require_nonempty(value: &str, field: &str) -> Result<(), Error> {
    if value.trim().is_empty() {
        Err(Error::InvalidArgument(format!("{field} must be non-empty")))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_server_lease_duration_is_a_protocol_error() {
        assert!(matches!(
            conservative_deadline(Instant::now(), 0),
            Err(Error::Protocol(_))
        ));
    }
}
