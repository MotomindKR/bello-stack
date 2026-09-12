use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use robstride_emulator::config::Config;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(name = "robstride-emulator")]
struct Args {
    /// Emulator configuration. Defaults to this component's vcan example.
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
    let config_path = args
        .config
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("config/vcan.yaml"));
    let config = Config::load(&config_path)?;
    if args.check {
        println!("{}: configuration is valid", config_path.display());
        return Ok(());
    }
    #[cfg(target_os = "linux")]
    return robstride_emulator::runtime::run(config);
    #[cfg(not(target_os = "linux"))]
    {
        drop(config);
        anyhow::bail!("SocketCAN emulation requires Linux")
    }
}
