use std::sync::Arc;

use tokio::sync::{broadcast, watch};

use super::{ResponseStream, conversion};
use crate::app::HardwareDaemon;
use crate::domain::{ComponentHealth, DeviceStatus, TelemetryPayload};
use bello_hardware_api::{WatchTelemetryRequest, WatchTelemetryResponse};

pub fn watch(
    daemon: Arc<HardwareDaemon>,
    filter: WatchTelemetryRequest,
    mut shutdown: watch::Receiver<bool>,
) -> ResponseStream<WatchTelemetryResponse> {
    let mut source = daemon.telemetry.subscribe();
    Box::pin(async_stream::stream! {
        'events: loop {
            let (event, force_delivery) = tokio::select! {
                biased;
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        loop {
                            let (event, force_delivery) = match source.try_recv() {
                                Ok(event) => (event, false),
                                Err(broadcast::error::TryRecvError::Lagged(dropped)) => {
                                    (subscriber_lagged(&daemon, dropped), true)
                                }
                                Err(
                                    broadcast::error::TryRecvError::Empty
                                    | broadcast::error::TryRecvError::Closed,
                                ) => break,
                            };
                            if force_delivery || selected(&filter, &event.payload) {
                                yield Ok(conversion::telemetry_event(
                                    event,
                                    daemon.telemetry.monotonic_ns(),
                                ));
                            }
                        }
                        break 'events;
                    }
                    continue 'events;
                }
                event = source.recv() => match event {
                    Ok(event) => (event, false),
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(dropped)) => {
                        (subscriber_lagged(&daemon, dropped), true)
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            };
            if force_delivery || selected(&filter, &event.payload) {
                yield Ok(conversion::telemetry_event(event, daemon.telemetry.monotonic_ns()));
            }
        }
    })
}

fn selected(filter: &WatchTelemetryRequest, payload: &TelemetryPayload) -> bool {
    match payload {
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
    }
}

fn subscriber_lagged(daemon: &HardwareDaemon, dropped: u64) -> crate::domain::TelemetryEvent {
    daemon
        .telemetry
        .event(TelemetryPayload::Health(ComponentHealth {
            component: "grpc_telemetry".into(),
            status: DeviceStatus {
                state: crate::domain::DeviceState::Degraded,
                detail: format!("subscriber lagged; dropped {dropped} events"),
                transition_count: 0,
            },
            dropped_events: dropped,
        }))
}
