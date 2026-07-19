use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Result, anyhow};
use tokio::sync::watch;
use tonic::transport::{Server, server::TcpIncoming};

use super::HardwareDaemon;
use crate::api::grpc::HardwareGrpc;
use crate::config::ResolvedTopology;
use crate::proto::hardware_service_server::HardwareServiceServer;
use crate::telemetry::Telemetry;

pub fn check(config_path: impl AsRef<Path>) -> Result<()> {
    let config_path = config_path.as_ref();
    ResolvedTopology::load(config_path)?;
    println!("{}: configuration is valid", config_path.display());
    Ok(())
}

pub async fn run(config_path: impl AsRef<Path>) -> Result<()> {
    let topology = ResolvedTopology::load(config_path)?;
    let listen = topology.grpc().listen();
    if !listen.ip().is_loopback() {
        tracing::warn!(%listen, "gRPC control is exposed without transport authentication");
    }
    let incoming = TcpIncoming::bind(listen)
        .map_err(|error| anyhow!("binding gRPC listener {listen}: {error}"))?;
    let mut signals = ShutdownSignals::install()?;
    let telemetry = Telemetry::new(topology.telemetry_capacity());
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let (observer_shutdown_tx, observer_shutdown_rx) = watch::channel(false);
    let rerun =
        crate::rerun_sink::start(topology.rerun(), telemetry.clone(), observer_shutdown_rx)?;
    let daemon = match HardwareDaemon::start(topology, telemetry) {
        Ok(daemon) => daemon,
        Err(error) => {
            let _ = observer_shutdown_tx.send(true);
            stop_rerun(rerun).await;
            return Err(error);
        }
    };
    let watchdog = tokio::spawn(HardwareDaemon::lease_watchdog(
        Arc::downgrade(&daemon),
        shutdown_rx.clone(),
    ));

    tracing::info!(%listen, "Bello hardware daemon ready");
    let server_result = {
        let server = Server::builder()
            .add_service(HardwareServiceServer::new(HardwareGrpc::new(
                daemon.clone(),
                shutdown_rx.clone(),
            )))
            .serve_with_incoming_shutdown(incoming, wait_for_shutdown(shutdown_rx));
        tokio::pin!(server);
        tokio::select! {
            result = server.as_mut() => {
                let _ = shutdown_tx.send(true);
                daemon.shutdown();
                result.map_err(anyhow::Error::from)
            },
            signal = signals.wait() => {
                let _ = shutdown_tx.send(true);
                daemon.shutdown();
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

    let _ = shutdown_tx.send(true);
    daemon.shutdown();
    let _ = watchdog.await;
    drop(daemon);
    let _ = observer_shutdown_tx.send(true);
    stop_rerun(rerun).await;
    server_result
}

async fn stop_rerun(rerun: crate::rerun_sink::RerunWorker) {
    rerun.join().await;
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
