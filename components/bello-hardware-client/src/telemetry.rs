use bello_hardware_api::watch_telemetry_response::Payload;
use bello_hardware_api::{WatchTelemetryRequest, WatchTelemetryResponse};
use tonic::Streaming;

use crate::{Error, HardwareClient};

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
        request: WatchTelemetryRequest,
    ) -> Result<Self, Error> {
        let mut grpc = client.inner.clone();
        let inner = grpc.watch_telemetry(request).await?.into_inner();
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
