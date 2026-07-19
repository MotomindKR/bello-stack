use std::sync::Arc;

use tokio::sync::watch;

use super::{ResponseStream, conversion};
use crate::app::HardwareDaemon;
use crate::domain::{ComponentHealth, DeviceStatus, TelemetryPayload};
use crate::proto::{WatchTelemetryRequest, WatchTelemetryResponse};

pub fn watch(
    daemon: Arc<HardwareDaemon>,
    filter: WatchTelemetryRequest,
    mut shutdown: watch::Receiver<bool>,
) -> ResponseStream<WatchTelemetryResponse> {
    let mut source = daemon.telemetry.subscribe();
    Box::pin(async_stream::stream! {
        loop {
            let (event, force_delivery) = tokio::select! {
                biased;
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                    continue;
                }
                event = source.recv() => match event {
                    Ok(event) => (event, false),
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(dropped)) => {
                        (daemon.telemetry.event(TelemetryPayload::Health(ComponentHealth {
                            component: "grpc_telemetry".into(),
                            status: DeviceStatus {
                                state: crate::domain::DeviceState::Degraded,
                                detail: format!("subscriber lagged; dropped {dropped} events"),
                                transition_count: 0,
                            },
                            dropped_events: dropped,
                        })), true)
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            };
            let selected = match &event.payload {
                TelemetryPayload::Motor(value) => {
                    filter.include_motors
                        && (filter.groups.is_empty()
                            || filter.groups.iter().any(|group| group == value.group.as_str()))
                }
                TelemetryPayload::Imu(value) => {
                    filter.include_imu
                        && (filter.imus.is_empty()
                            || filter.imus.iter().any(|imu| imu == value.imu.as_str()))
                }
                TelemetryPayload::Group(value) => {
                    filter.groups.is_empty()
                        || filter.groups.iter().any(|group| group == value.group.as_str())
                }
                TelemetryPayload::CommandPolicy(value) => {
                    filter.include_command_policy_events
                        && (filter.groups.is_empty()
                            || filter.groups.iter().any(|group| group == value.group.as_str()))
                }
                TelemetryPayload::Health(_) => filter.include_health,
            };
            if force_delivery || selected {
                yield Ok(conversion::telemetry_event(event, daemon.telemetry.monotonic_ns()));
            }
        }
    })
}
