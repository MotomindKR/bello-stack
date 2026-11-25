use std::env;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};

use crate::config::ResolvedTopology;
use crate::domain::MotorName;
use crate::hardware::can::BusManager;
use crate::hardware::can::BusSpec;
use crate::maintenance;
use crate::telemetry::Telemetry;

#[derive(Serialize, Deserialize)]
pub(super) struct PendingWrite {
    pub id: String,
    pub motor: String,
    pub bus: String,
    pub address: u8,
    pub uuid: [u8; 8],
    pub expected_version: String,
    pub zero: bool,
}

pub(super) struct Journal {
    path: PathBuf,
    _lock: File,
}

pub(super) fn state_directory() -> Result<PathBuf> {
    let base = env::var_os("XDG_STATE_HOME")
        .or_else(|| env::var_os("LOCALAPPDATA"))
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| env::home_dir().map(|p| p.join(".local/state")))
        .context("cannot locate the motor configuration state directory")?;
    Ok(base.join("bello-hardwared"))
}

pub(super) fn journal_path(directory: &Path, bus: &BusSpec) -> PathBuf {
    let key = ring::digest::digest(&ring::digest::SHA256, bus.transport.label().as_bytes());
    let hex: String = key.as_ref().iter().map(|b| format!("{b:02x}")).collect();
    directory.join(format!("{hex}.pending"))
}

pub(super) fn check_clear(path: &Path) -> Result<()> {
    ensure!(
        !path.try_exists()?,
        "unverified motor configuration at {}. Run --verify-motor-configuration before starting control",
        path.display()
    );
    Ok(())
}

impl Journal {
    fn lock(path: &Path) -> Result<Self> {
        let lock = File::options()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path.with_added_extension("lock"))
            .context("opening motor configuration journal lock")?;
        lock.try_lock()
            .context("another process owns this motor configuration journal")?;
        Ok(Self {
            path: path.into(),
            _lock: lock,
        })
    }

    pub(super) fn begin(path: &Path, record: &PendingWrite) -> Result<Self> {
        fs::create_dir_all(path.parent().context("missing journal directory")?)?;
        let journal = Self::lock(path)?;
        let mut tmp =
            tempfile::NamedTempFile::new_in(path.parent().context("missing journal directory")?)?;
        tmp.write_all(serde_yaml::to_string(record)?.as_bytes())?;
        tmp.as_file().sync_all()?;
        tmp.persist_noclobber(path)
            .context("recording motor configuration before write")?;
        sync_parent(path)?;
        Ok(journal)
    }

    pub(super) fn clear(&self) -> Result<()> {
        fs::remove_file(&self.path)?;
        sync_parent(&self.path)
    }
}

fn sync_parent(path: &Path) -> Result<()> {
    #[cfg(unix)]
    File::open(path.parent().context("missing journal directory")?)?.sync_all()?;
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

pub fn verify_motor_configuration(config: impl AsRef<Path>) -> Result<()> {
    let topology = ResolvedTopology::load(config)?;
    topology.validate_platform()?;
    let directory = state_directory()?;
    let pending = topology
        .buses()
        .map(|(_, bus)| journal_path(&directory, bus))
        .map(|p| Ok(p.try_exists()?.then_some(p)))
        .collect::<Result<Vec<_>>>()?;
    let pending: Vec<_> = pending.into_iter().flatten().collect();
    ensure!(
        !pending.is_empty(),
        "no pending writes on this deployment's CAN buses"
    );
    let buses = BusManager::start(&topology, Telemetry::new(16))?;
    for path in pending {
        let journal = Journal::lock(&path)?;
        let record: PendingWrite = serde_yaml::from_reader(
            File::open(&path).context("opening pending motor configuration")?,
        )?;
        let motor = &topology
            .motor(&MotorName::from(record.motor.as_str()))?
            .spec;
        let handle = buses.handle(&motor.address.bus)?;
        ensure!(
            journal_path(&directory, &handle.spec) == path && motor.address.id == record.address,
            "motor transport or address differs from pending write"
        );
        let until = Instant::now() + Duration::from_secs(3);
        while !handle.status().state.is_online() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(20));
        }
        let bus = handle.reserve_maintenance()?;
        maintenance::verify_after_interruption(
            &bus,
            motor,
            handle.spec.host_id,
            record.uuid,
            &record.expected_version,
            record.zero,
        )?;
        journal.clear()?;
        println!(
            "Verified motor configuration {} for {}",
            record.id, record.motor
        );
    }
    Ok(())
}
