use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use robstride_emulator::config::Config;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(name = "robstride-emulator")]
struct Args {
    /// Emulator configuration. Defaults to the bundled Milly UDP profile.
    #[arg(short, long)]
    config: Option<PathBuf>,
    #[arg(long)]
    check: bool,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let args = Args::parse();
    let config = match args.config {
        Some(path) => Config::load(path)?,
        None => Config::parse(include_str!("../config/milly.yaml"))?,
    };
    if args.check {
        println!("configuration is valid");
        return Ok(());
    }
    robstride_emulator::runtime::run(config)
}
