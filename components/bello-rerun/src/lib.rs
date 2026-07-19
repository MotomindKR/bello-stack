mod backoff;
mod config;
mod mapping;
mod observer;
mod timeline;
mod writer;

use std::path::Path;

use anyhow::{Result, anyhow};

pub use config::Config;

pub fn check(config_path: impl AsRef<Path>) -> Result<()> {
    let config_path = config_path.as_ref();
    Config::load(config_path)?;
    println!("{}: configuration is valid", config_path.display());
    Ok(())
}

/// Runs the process-lifetime observer entrypoint.
///
/// If the Rerun SDK cannot finish within the shutdown deadline, its isolated
/// writer thread is detached so the `bello-rerun` process can exit. Embedders
/// that keep their process alive after this future returns should account for
/// that exceptional detached-thread case.
pub async fn run(config_path: impl AsRef<Path>) -> Result<()> {
    let config = Config::load(config_path.as_ref())?;
    let writer = writer::RerunWriter::start(&config.output)?;

    let result = tokio::select! {
        result = observer::observe(&config, &writer) => {
            match result {
                Ok(()) => Err(anyhow!("telemetry observer stopped unexpectedly")),
                Err(error) => Err(error),
            }
        }
        signal = wait_for_shutdown_signal() => signal,
    };

    writer.shutdown().await;
    result
}

#[cfg(unix)]
async fn wait_for_shutdown_signal() -> Result<()> {
    let mut interrupt = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    tokio::select! {
        received = interrupt.recv() => received
            .ok_or_else(|| anyhow!("SIGINT handler closed unexpectedly"))
            .map(|_| ()),
        received = terminate.recv() => received
            .ok_or_else(|| anyhow!("SIGTERM handler closed unexpectedly"))
            .map(|_| ()),
    }
}

#[cfg(not(unix))]
async fn wait_for_shutdown_signal() -> Result<()> {
    tokio::signal::ctrl_c().await.map_err(anyhow::Error::from)
}
