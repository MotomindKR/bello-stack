mod support;

use std::fs;
use std::net::{SocketAddr, TcpListener};
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use support::process::ManagedProcess;

#[test]
fn generated_runtime_configs_are_valid() -> Result<()> {
    let directory = tempfile::tempdir()?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    fs::write(
        &hardwared_config,
        support::config::hardwared("vcan_test", "127.0.0.1:50052".parse()?),
    )?;
    fs::write(&emulator_config, support::config::emulator("vcan_test"))?;

    bello_hardwared::check(&hardwared_config)?;
    robstride_emulator::config::Config::load(&emulator_config)?;
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "requires a host vcan interface"]
async fn control_fault_and_recovery_over_vcan() -> Result<()> {
    let interface = selected_interface()?;
    require_vcan(&interface)?;
    let listen = available_loopback_address()?;
    let directory = tempfile::tempdir().context("creating e2e configuration directory")?;
    let hardwared_config = directory.path().join("hardwared.yaml");
    let emulator_config = directory.path().join("emulator.yaml");
    fs::write(
        &hardwared_config,
        support::config::hardwared(&interface, listen),
    )?;
    fs::write(&emulator_config, support::config::emulator(&interface))?;

    let mut emulator = ManagedProcess::spawn(
        "robstride-emulator",
        env!("CARGO_BIN_EXE_robstride-emulator-e2e-fixture"),
        [&emulator_config],
    )?;
    let mut hardwared = ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [&hardwared_config],
    )?;
    let endpoint = format!("http://{listen}");
    let client = support::scenario::connect(
        &endpoint,
        Duration::from_secs(3),
        &mut hardwared,
        &mut emulator,
    )
    .await?;
    support::scenario::run(
        client,
        Duration::from_secs(8),
        &mut hardwared,
        &mut emulator,
    )
    .await
}

fn selected_interface() -> Result<String> {
    let interface = std::env::var("BELLO_E2E_CAN_INTERFACE").unwrap_or_else(|_| "vcan0".into());
    if interface.is_empty()
        || interface.len() > 15
        || !interface
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
    {
        bail!("BELLO_E2E_CAN_INTERFACE must be a valid Linux interface name");
    }
    Ok(interface)
}

fn require_vcan(interface: &str) -> Result<()> {
    let path = PathBuf::from("/sys/class/net").join(interface);
    if !path.is_dir() {
        bail!(
            "SocketCAN interface {interface} does not exist; create and bring up a vcan interface first"
        );
    }
    let hardware_type = read_trimmed(path.join("type"))?;
    if hardware_type != "280" {
        bail!("interface {interface} is not a CAN interface (ARPHRD type {hardware_type})");
    }
    let flags = read_trimmed(path.join("flags"))?;
    let flags = u32::from_str_radix(flags.trim_start_matches("0x"), 16)
        .with_context(|| format!("parsing flags for interface {interface}"))?;
    if flags & 1 == 0 {
        bail!("SocketCAN interface {interface} is not up");
    }
    Ok(())
}

fn read_trimmed(path: impl AsRef<Path>) -> Result<String> {
    let path = path.as_ref();
    Ok(fs::read_to_string(path)
        .with_context(|| format!("reading {}", path.display()))?
        .trim()
        .to_owned())
}

fn available_loopback_address() -> Result<SocketAddr> {
    let listener = TcpListener::bind("127.0.0.1:0").context("reserving a loopback gRPC port")?;
    let address = listener.local_addr()?;
    drop(listener);
    Ok(address)
}
