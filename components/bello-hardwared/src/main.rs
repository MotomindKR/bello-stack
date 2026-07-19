use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
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
    if args.check {
        bello_hardwared::check(config_path)
    } else {
        bello_hardwared::run(config_path).await
    }
}
