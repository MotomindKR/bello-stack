use std::sync::Arc;

use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;

use super::{ResponseStream, conversion};
use crate::app::HardwareDaemon;
use crate::domain::{ComponentHealth, TelemetryEvent, TelemetryPayload};
use crate::proto::{WatchTelemetryRequest, WatchTelemetryResponse};

pub fn watch(
    daemon: Arc<HardwareDaemon>,
    filter: WatchTelemetryRequest,
) -> ResponseStream<WatchTelemetryResponse> {
    let mut source = daemon.telemetry.subscribe();
    let (tx, rx) = mpsc::channel(1024);
    tokio::spawn(async move {
        loop {
            let mut event = match source.recv().await {
                Ok(event) => event,
                Err(tokio::sync::broadcast::error::RecvError::Lagged(dropped)) => {
                    let event = TelemetryEvent {
                        monotonic_ns: daemon.telemetry.monotonic_ns(),
                        payload: TelemetryPayload::Health(ComponentHealth {
                            component: "grpc_telemetry".into(),
                            status: crate::domain::DeviceState::Degraded,
                            detail: format!("subscriber lagged; dropped {dropped} events"),
                            transition_count: 0,
                            dropped_events: dropped,
                        }),
                    };
                    if tx
                        .send(Ok(conversion::telemetry_event(
                            event,
                            daemon.telemetry.monotonic_ns(),
                        )))
                        .await
                        .is_err()
                    {
                        break;
                    }
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
            };
            if let TelemetryPayload::Group(group) = &mut event.payload {
                group.lease_owner = daemon.lease_owner(&group.group);
            }
            let selected = match &event.payload {
                TelemetryPayload::Motor(value) => {
                    filter.include_motors
                        && (filter.groups.is_empty()
                            || filter
                                .groups
                                .iter()
                                .any(|group| group == value.group.as_str()))
                }
                TelemetryPayload::Imu(value) => {
                    filter.include_imu
                        && (filter.imus.is_empty()
                            || filter.imus.iter().any(|imu| imu == value.imu.as_str()))
                }
                TelemetryPayload::Group(value) => {
                    filter.groups.is_empty()
                        || filter
                            .groups
                            .iter()
                            .any(|group| group == value.group.as_str())
                }
                TelemetryPayload::CommandPolicy(value) => {
                    filter.include_command_policy_events
                        && (filter.groups.is_empty()
                            || filter
                                .groups
                                .iter()
                                .any(|group| group == value.group.as_str()))
                }
                TelemetryPayload::Health(_) => filter.include_health,
            };
            if selected
                && tx
                    .send(Ok(conversion::telemetry_event(
                        event,
                        daemon.telemetry.monotonic_ns(),
                    )))
                    .await
                    .is_err()
            {
                break;
            }
        }
    });
    Box::pin(ReceiverStream::new(rx))
}
