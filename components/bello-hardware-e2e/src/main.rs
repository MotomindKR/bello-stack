mod process;
mod scenario;

use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::Parser;
use process::ManagedProcess;

#[derive(Debug, Parser)]
#[command(name = "bello-hardware-e2e")]
struct Args {
    #[arg(long, default_value_os_t = sibling_binary("bello-hardwared"))]
    hardwared: PathBuf,
    #[arg(long, default_value_os_t = sibling_binary("robstride-emulator"))]
    emulator: PathBuf,
    #[arg(long, default_value_os_t = component_file("config/hardwared.yaml"))]
    hardwared_config: PathBuf,
    #[arg(long, default_value_os_t = component_file("config/emulator.yaml"))]
    emulator_config: PathBuf,
    #[arg(long, default_value = "vcan0")]
    interface: String,
    #[arg(long, default_value = "http://127.0.0.1:50052")]
    endpoint: String,
    #[arg(long, default_value_t = 8)]
    timeout_seconds: u64,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    require_interface(&args.interface)?;
    build_runtime_binaries()?;
    require_file(&args.hardwared, "hardwared binary")?;
    require_file(&args.emulator, "emulator binary")?;
    require_file(&args.hardwared_config, "hardwared configuration")?;
    require_file(&args.emulator_config, "emulator configuration")?;

    let _emulator = ManagedProcess::spawn(
        "robstride-emulator",
        &args.emulator,
        ["--config".as_ref(), args.emulator_config.as_os_str()],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        &args.hardwared,
        ["--config".as_ref(), args.hardwared_config.as_os_str()],
    )?;
    let timeout = Duration::from_secs(args.timeout_seconds);
    let client = scenario::connect(&args.endpoint, Duration::from_secs(3)).await?;
    hardwared.ensure_running()?;
    scenario::run(client, timeout).await
}

fn sibling_binary(name: &str) -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|path| path.parent().map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from("target/debug"))
        .join(name)
}

fn component_file(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative)
}

fn build_runtime_binaries() -> Result<()> {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let status = Command::new(cargo)
        .args(["build", "-p", "bello-hardwared", "-p", "robstride-emulator"])
        .status()
        .context("building e2e runtime binaries")?;
    if !status.success() {
        bail!("failed to build e2e runtime binaries");
    }
    Ok(())
}

fn require_interface(interface: &str) -> Result<()> {
    let path = PathBuf::from("/sys/class/net").join(interface);
    if !path.exists() {
        bail!(
            "SocketCAN interface {interface} does not exist; create and bring up a vcan interface first"
        );
    }
    Ok(())
}

fn require_file(path: &std::path::Path, description: &str) -> Result<()> {
    if !path.is_file() {
        bail!("{description} not found at {}", path.display());
    }
    path.canonicalize()
        .with_context(|| format!("resolving {description} {}", path.display()))?;
    Ok(())
}
