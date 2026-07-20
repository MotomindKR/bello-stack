use std::future::Future;
use std::path::Path;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Result, anyhow};
use tokio::sync::watch;

use super::HardwareDaemon;
use super::listener::GrpcIncoming;
use crate::api::grpc::GrpcListenSpec;
use crate::api::grpc::HardwareGrpc;
use crate::config::ResolvedTopology;
use crate::telemetry::Telemetry;
use bello_hardware_api::hardware_service_server::HardwareServiceServer;

pub fn check(config_path: impl AsRef<Path>) -> Result<()> {
    let config_path = config_path.as_ref();
    ResolvedTopology::load(config_path)?;
    println!("{}: configuration is valid", config_path.display());
    Ok(())
}

pub async fn run(config_path: impl AsRef<Path>) -> Result<()> {
    let topology = ResolvedTopology::load(config_path)?;
    let listen = topology.grpc().listen().clone();
    match &listen {
        GrpcListenSpec::Tcp(address)
            if !address.ip().is_loopback() && !topology.grpc().uses_tls() =>
        {
            tracing::warn!(%address, "gRPC control is exposed without transport encryption or authentication");
        }
        GrpcListenSpec::Tcp(address)
            if !address.ip().is_loopback() && !topology.grpc().requires_client_certificate() =>
        {
            tracing::warn!(%address, "gRPC TLS is enabled without client authentication");
        }
        _ => {}
    }
    let incoming = GrpcIncoming::bind(&listen)?;
    let mut server = topology.grpc().server()?;
    let mut signals = ShutdownSignals::install()?;
    let telemetry = Telemetry::new(topology.telemetry_capacity());
    let (server_shutdown_tx, server_shutdown_rx) = watch::channel(false);
    let (watchdog_shutdown_tx, watchdog_shutdown_rx) = watch::channel(false);
    let daemon = HardwareDaemon::start(topology, telemetry)?;
    let watchdog = tokio::spawn(HardwareDaemon::lease_watchdog(
        Arc::downgrade(&daemon),
        watchdog_shutdown_rx,
    ));

    tracing::info!(?listen, "Bello hardware daemon ready");
    let server_result = {
        type ServerFuture =
            Pin<Box<dyn Future<Output = Result<(), tonic::transport::Error>> + Send + 'static>>;
        let service = HardwareServiceServer::new(HardwareGrpc::new(
            daemon.clone(),
            server_shutdown_rx.clone(),
        ));
        let mut server: ServerFuture = match incoming {
            GrpcIncoming::Tcp(incoming) => Box::pin(
                server
                    .add_service(service)
                    .serve_with_incoming_shutdown(incoming, wait_for_shutdown(server_shutdown_rx)),
            ),
            #[cfg(unix)]
            GrpcIncoming::Unix { incoming, socket } => Box::pin(async move {
                let _socket = socket;
                server
                    .add_service(service)
                    .serve_with_incoming_shutdown(incoming, wait_for_shutdown(server_shutdown_rx))
                    .await
            }),
        };
        tokio::select! {
            result = server.as_mut() => {
                begin_shutdown(&daemon, &watchdog_shutdown_tx, &server_shutdown_tx);
                result.map_err(anyhow::Error::from)
            },
            signal = signals.wait() => {
                begin_shutdown(&daemon, &watchdog_shutdown_tx, &server_shutdown_tx);
                let drain_result = match tokio::time::timeout(Duration::from_secs(2), server.as_mut()).await {
                    Ok(result) => result.map_err(anyhow::Error::from),
                    Err(_) => {
                        tracing::warn!("gRPC drain deadline expired; forcing server shutdown");
                        Ok(())
                    }
                };
                signal.and(drain_result)
            }
        }
    };

    begin_shutdown(&daemon, &watchdog_shutdown_tx, &server_shutdown_tx);
    let _ = watchdog.await;
    drop(daemon);
    server_result
}

/// Stops lease expiry, publishes and joins final group state, then lets gRPC
/// streams drain that queued telemetry. Each operation is idempotent.
fn begin_shutdown(
    daemon: &HardwareDaemon,
    watchdog_shutdown: &watch::Sender<bool>,
    server_shutdown: &watch::Sender<bool>,
) {
    let _ = watchdog_shutdown.send(true);
    daemon.shutdown();
    let _ = server_shutdown.send(true);
}

#[cfg(unix)]
struct ShutdownSignals {
    interrupt: tokio::signal::unix::Signal,
    terminate: tokio::signal::unix::Signal,
}

#[cfg(unix)]
impl ShutdownSignals {
    fn install() -> Result<Self> {
        Ok(Self {
            interrupt: tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?,
            terminate: tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?,
        })
    }

    async fn wait(&mut self) -> Result<()> {
        tokio::select! {
            received = self.interrupt.recv() => received
                .ok_or_else(|| anyhow!("SIGINT handler closed unexpectedly"))
                .map(|_| ()),
            received = self.terminate.recv() => received
            .ok_or_else(|| anyhow!("SIGTERM handler closed unexpectedly"))
            .map(|_| ()),
        }
    }
}

#[cfg(not(unix))]
struct ShutdownSignals;

#[cfg(not(unix))]
impl ShutdownSignals {
    fn install() -> Result<Self> {
        Ok(Self)
    }

    async fn wait(&mut self) -> Result<()> {
        tokio::signal::ctrl_c().await.map_err(anyhow::Error::from)
    }
}

async fn wait_for_shutdown(mut shutdown: watch::Receiver<bool>) {
    while !*shutdown.borrow() {
        if shutdown.changed().await.is_err() {
            break;
        }
    }
}
