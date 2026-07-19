use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use bello_hardware_api::{WatchTelemetryRequest, hardware_service_client::HardwareServiceClient};

use crate::backoff::Backoff;
use crate::config::Config;
use crate::writer::RerunWriter;

const STREAM_ESTABLISH_TIMEOUT: Duration = Duration::from_secs(5);

pub(crate) async fn observe(config: &Config, writer: &RerunWriter) -> Result<()> {
    let endpoint = config.endpoint()?;
    let mut backoff = Backoff::new(
        Duration::from_millis(config.reconnect.initial_delay_ms),
        Duration::from_millis(config.reconnect.maximum_delay_ms),
    );
    let mut ever_connected = false;

    loop {
        let stream_result = match tokio::time::timeout(STREAM_ESTABLISH_TIMEOUT, async {
            let channel = endpoint
                .connect()
                .await
                .context("connecting to bello-hardwared")?;
            let mut client = HardwareServiceClient::new(channel);
            client
                .watch_telemetry(all_telemetry_request())
                .await
                .context("starting WatchTelemetry")
                .map(tonic::Response::into_inner)
        })
        .await
        {
            Ok(result) => result,
            Err(_) => Err(anyhow!(
                "timed out establishing WatchTelemetry after {STREAM_ESTABLISH_TIMEOUT:?}"
            )),
        };

        let mut stream = match stream_result {
            Ok(stream) => stream,
            Err(error) => {
                let delay = backoff.next_delay();
                tracing::warn!(%error, ?delay, "hardware telemetry unavailable; retrying");
                tokio::time::sleep(delay).await;
                continue;
            }
        };

        let connection = writer.connection_opened(ever_connected)?;
        ever_connected = true;
        tracing::info!(endpoint = %config.hardware_endpoint, "watching hardware telemetry");

        let mut received_event = false;
        let disconnect_detail = loop {
            match stream.message().await {
                Ok(Some(response)) => {
                    if !received_event {
                        backoff.reset();
                        received_event = true;
                    }
                    writer.telemetry(connection, response)?;
                }
                Ok(None) => break "WatchTelemetry stream closed".to_owned(),
                Err(error) => break format!("WatchTelemetry stream failed: {error}"),
            }
        };

        writer.connection_lost(disconnect_detail.clone())?;
        let delay = backoff.next_delay();
        tracing::warn!(detail = %disconnect_detail, ?delay, "hardware telemetry disconnected; retrying");
        tokio::time::sleep(delay).await;
    }
}

fn all_telemetry_request() -> WatchTelemetryRequest {
    WatchTelemetryRequest {
        groups: Vec::new(),
        include_motors: true,
        include_imu: true,
        include_health: true,
        imus: Vec::new(),
        include_command_policy_events: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subscription_requests_every_telemetry_category() {
        let request = all_telemetry_request();
        assert!(request.groups.is_empty());
        assert!(request.imus.is_empty());
        assert!(request.include_motors);
        assert!(request.include_imu);
        assert!(request.include_health);
        assert!(request.include_command_policy_events);
    }
}
