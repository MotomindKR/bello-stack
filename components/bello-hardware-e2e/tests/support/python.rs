use std::{env, path::Path, path::PathBuf, process::Command};

use anyhow::{Context, Result};

fn components() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("E2E component must have a components parent")
}

pub fn sdk_package() -> PathBuf {
    components().join("bello-hardware-python")
}

pub fn command() -> Result<Command> {
    let sources = [
        sdk_package().join("src"),
        components().join("bello-hardware-api/src"),
    ];
    let path = env::join_paths(sources).context("building Python smoke client import path")?;
    let mut command = Command::new("python3");
    command.env("PYTHONPATH", path);
    Ok(command)
}
