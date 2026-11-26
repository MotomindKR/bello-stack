use std::env;
use std::fs::{self, File};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::config::ResolvedTopology;

pub(super) fn state_directory() -> Result<PathBuf> {
    let base = env::var_os("XDG_STATE_HOME")
        .or_else(|| env::var_os("LOCALAPPDATA"))
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| env::home_dir().map(|p| p.join(".local/state")))
        .context("cannot locate the motor configuration state directory")?;
    Ok(base.join("bello-hardwared"))
}

pub(super) fn claim_transports(directory: &Path, topology: &ResolvedTopology) -> Result<Vec<File>> {
    fs::create_dir_all(directory).context("creating hardwared state directory")?;
    topology
        .buses()
        .map(|(_, bus)| {
            let digest =
                ring::digest::digest(&ring::digest::SHA256, bus.transport.label().as_bytes());
            let key: String = digest.as_ref().iter().map(|b| format!("{b:02x}")).collect();
            let path = directory.join(format!("{key}.lock"));
            let lock = File::options()
                .read(true)
                .write(true)
                .create(true)
                .truncate(false)
                .open(path)?;
            lock.try_lock().with_context(|| {
                format!("another hardwared process owns {}", bus.transport.label())
            })?;
            Ok(lock)
        })
        .collect()
}
