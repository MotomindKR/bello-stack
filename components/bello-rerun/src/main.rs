use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(version, about)]
struct Arguments {
    /// Rerun observer configuration. Defaults to this component's example config.
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Parse and validate configuration, then exit without connecting or creating output.
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

    let arguments = Arguments::parse();
    let config_path = arguments
        .config
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("config/bello.yaml"));
    if arguments.check {
        bello_rerun::check(config_path)
    } else {
        bello_rerun::run(config_path).await
    }
}
