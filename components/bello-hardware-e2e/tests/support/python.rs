use std::{env, path::Path, path::PathBuf, process::Command};

use anyhow::{Context, Result};

fn python_bindings() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("E2E component must have a components parent")
        .join("bello-hardware-api/bindings/python")
}

pub fn sdk_package() -> PathBuf {
    python_bindings().join("client")
}

pub fn command() -> Result<Command> {
    let sources = [sdk_package().join("src"), python_bindings().join("api/src")];
    let path = env::join_paths(sources).context("building Python smoke client import path")?;
    let mut command = Command::new("python3");
    command.env("PYTHONPATH", path);
    Ok(command)
}
