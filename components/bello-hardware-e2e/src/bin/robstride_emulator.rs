use std::path::PathBuf;

use anyhow::{Context, Result};
use robstride_emulator::config::Config;

fn main() -> Result<()> {
    let config_path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: robstride-emulator-e2e-fixture CONFIG")?;
    let config = Config::load(config_path)?;
    robstride_emulator::runtime::run(config)
}
