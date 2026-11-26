use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use tracing_subscriber::EnvFilter;

#[derive(Parser, Debug)]
#[command(version, about)]
struct Arguments {
    /// Deployment configuration composed with the canonical hardware description.
    #[arg(short, long)]
    config: PathBuf,
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
    if args.check {
        bello_hardwared::check(args.config)
    } else {
        bello_hardwared::run(args.config).await
    }
}
