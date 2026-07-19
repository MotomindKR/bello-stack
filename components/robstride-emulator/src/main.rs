use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use robstride_emulator::config::Config;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(name = "robstride-emulator")]
struct Args {
    #[arg(short, long, default_value = "config/vcan.yaml")]
    config: PathBuf,
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
    let config = Config::load(&args.config)?;
    if args.check {
        println!("{}: configuration is valid", args.config.display());
        return Ok(());
    }
    robstride_emulator::runtime::run(config)
}
