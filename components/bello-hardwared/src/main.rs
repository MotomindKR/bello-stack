use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use bello_hardwared::api::grpc::HardwareGrpc;
use bello_hardwared::app::HardwareDaemon;
use bello_hardwared::config::ResolvedTopology;
use bello_hardwared::proto::hardware_service_server::HardwareServiceServer;
use clap::Parser;
use tonic::transport::Server;
use tracing::info;
use tracing_subscriber::EnvFilter;

#[derive(Parser, Debug)]
#[command(version, about)]
struct Arguments {
    /// Unified hardware configuration. Defaults to this component's example config.
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Parse and validate configuration, then exit without touching hardware.
    #[arg(long)]
    check: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let args = Arguments::parse();
    let config_path = args
        .config
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("config/bello.yaml"));
    let topology = ResolvedTopology::load(&config_path)?;
    if args.check {
        println!("{}: configuration is valid", config_path.display());
        return Ok(());
    }
    let daemon = HardwareDaemon::start(topology)?;
    let listen = daemon.topology.grpc().listen();
    bello_hardwared::rerun_sink::start(daemon.topology.rerun(), daemon.telemetry.clone())?;
    tokio::spawn(HardwareDaemon::lease_watchdog(Arc::downgrade(&daemon)));
    info!(%listen, "Bello hardware daemon ready");
    Server::builder()
        .add_service(HardwareServiceServer::new(HardwareGrpc::new(daemon)))
        .serve_with_shutdown(listen, async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}
