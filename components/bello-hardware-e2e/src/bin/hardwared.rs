use std::path::PathBuf;

use anyhow::{Context, Result};

#[tokio::main]
async fn main() -> Result<()> {
    let config = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: hardwared-e2e-fixture CONFIG")?;
    bello_hardwared::run(config).await
}
