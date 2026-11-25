use std::env;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use ring::digest::{SHA256, digest};
use serde::Deserialize;

use super::{Link, Version};

const MAX_IMAGE_SIZE: usize = 0x80000;
const ACK_TIMEOUT: Duration = Duration::from_secs(2);
const ACK_ATTEMPTS: usize = 3;
const MAX_RESUMES: usize = 30;
const START: u8 = 11;
const INFO: u8 = 12;
const DATA: u8 = 13;
const END: u8 = 14;

pub(super) struct Image {
    pub(super) version: Version,
    bytes: Vec<u8>,
}

#[derive(Deserialize)]
struct Release {
    model: String,
    version: String,
    url: String,
    sha256: String,
}

impl Image {
    pub(super) fn prepare(model: &str, installed: Version, simulation: bool) -> Result<Self> {
        if simulation {
            let version = Version([0, installed.0[1], 0, 2]);
            require_upgrade(installed, version)?;
            return Ok(Self {
                version,
                bytes: format!("BELLO-SIM-FW/1\n{version}").into_bytes(),
            });
        }

        let cache = cache_directory()?;
        let catalog = cached_file(
            &cache,
            "https://raw.githubusercontent.com/MotomindKR/motor-setup-tool/3437f89e0969b394fc729ec5945d29065eef6985/src/firmware_catalog.json",
            "830b526562dfac39e079bc0f09767c81c60766a761ea5938cf77c5ac855c1bbc",
            64 * 1024,
        ).context("loading the pinned motor-setup-tool firmware catalog")?;
        let catalog: Vec<Release> = serde_json::from_slice(&catalog)?;
        let release = catalog
            .into_iter()
            .find(|release| release.model == model.replace('_', "-"))
            .with_context(|| format!("No verified firmware release for {model}"))?;
        let version: Version = release.version.parse()?;
        version.check_model(model)?;
        require_upgrade(installed, version)?;
        let bytes = cached_file(&cache, &release.url, &release.sha256, MAX_IMAGE_SIZE)
            .with_context(|| format!("loading {} firmware {}", release.model, release.version))?;
        Ok(Self { version, bytes })
    }

    // A missing END acknowledgement can mean the motor already rebooted. The
    // caller must verify the new application even when every END attempt timed out.
    pub(super) fn transfer(
        &self,
        link: &Link<'_>,
        uuid: [u8; 8],
        progress: &impl Fn(u32),
    ) -> Result<()> {
        let host = u16::from(link.host);
        require_ack(link, START, host, uuid)?;

        let count = self.bytes.len().div_ceil(8);
        let count_bytes = (count as u32).to_le_bytes();
        let mut info = [0; 8];
        info[..4].copy_from_slice(&(self.bytes.len() as u32).to_le_bytes());
        info[4..].copy_from_slice(&count_bytes);
        require_ack(link, INFO, host, info)?;

        let mut index = 0;
        let mut resumes = 0;
        let mut reported = 0;
        while index < count {
            let mut packet = [0xff; 8];
            let start = index * 8;
            let chunk = &self.bytes[start..(start + 8).min(self.bytes.len())];
            packet[..chunk.len()].copy_from_slice(chunk);
            let ack = exchange(link, DATA, index as u16, packet)?
                .with_context(|| format!("No OTA acknowledgment for packet {index}"))?;
            match ack.status {
                0 => index += 1,
                0x0f | 0xf0 => {
                    resumes += 1;
                    let next = u16::from_le_bytes([ack.data[0], ack.data[1]]) as usize;
                    ensure!(
                        next <= index && resumes <= MAX_RESUMES,
                        "Invalid or excessive OTA resume request for packet {next} at {index}"
                    );
                    index = next;
                }
                status => anyhow::bail!("OTA packet {index} rejected with status 0x{status:02x}"),
            }
            let percent = 95 * index as u32 / count as u32;
            if percent > reported {
                progress(percent);
                reported = percent;
            }
            thread::sleep(Duration::from_millis(1));
        }

        let mut end = [0; 8];
        end[..4].copy_from_slice(&count_bytes);
        if let Some(ack) = exchange(link, END, 0, end)? {
            ensure!(
                ack.status == 0,
                "OTA END rejected with status 0x{:02x}",
                ack.status
            );
        }
        Ok(())
    }
}

fn require_upgrade(installed: Version, target: Version) -> Result<()> {
    ensure!(
        installed < target,
        "Installed firmware {installed} is already at or newer than {target}"
    );
    Ok(())
}

struct Ack {
    status: u8,
    data: [u8; 8],
}

fn exchange(link: &Link<'_>, kind: u8, field: u16, data: [u8; 8]) -> Result<Option<Ack>> {
    for attempt in 0..ACK_ATTEMPTS {
        link.send(kind, field, data)?;
        if let Some(ack) = link.wait_for(ACK_TIMEOUT, |frame| {
            link.is_reply(frame, kind).then_some(Ack {
                status: (frame.id >> 16) as u8,
                data: frame.data,
            })
        })? {
            return Ok(Some(ack));
        }
        if attempt + 1 < ACK_ATTEMPTS {
            thread::sleep(Duration::from_millis(50));
        }
    }
    Ok(None)
}

fn require_ack(link: &Link<'_>, kind: u8, field: u16, data: [u8; 8]) -> Result<()> {
    let ack = exchange(link, kind, field, data)?.with_context(|| {
        format!("No OTA acknowledgment for type {kind} after {ACK_ATTEMPTS} attempts")
    })?;
    ensure!(
        ack.status == 0,
        "OTA type {kind} rejected with status 0x{:02x}",
        ack.status
    );
    Ok(())
}

fn cache_directory() -> Result<PathBuf> {
    let base = env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .filter(|path| path.is_absolute())
        .or_else(|| env::home_dir().map(|home| home.join(".cache")))
        .context("Cannot locate the firmware cache directory")?;
    Ok(base.join("bello-hardwared").join("firmware"))
}

fn cached_file(directory: &Path, url: &str, sha256: &str, maximum: usize) -> Result<Vec<u8>> {
    ensure!(
        sha256.len() == 64 && sha256.bytes().all(|b| b.is_ascii_hexdigit()),
        "invalid pinned SHA-256"
    );
    let valid = |bytes: &[u8]| {
        let actual: String = digest(&SHA256, bytes)
            .as_ref()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        !bytes.is_empty() && bytes.len() <= maximum && actual == sha256
    };
    fs::create_dir_all(directory).context("creating firmware cache directory")?;
    let path = directory.join(sha256);
    if let Ok(file) = File::open(&path) {
        let mut bytes = Vec::new();
        file.take((maximum + 1) as u64).read_to_end(&mut bytes)?;
        if valid(&bytes) {
            return Ok(bytes);
        }
    }
    let agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .new_agent();
    let bytes = agent
        .get(url)
        .call()?
        .body_mut()
        .with_config()
        .limit(maximum as u64)
        .read_to_vec()?;
    ensure!(
        valid(&bytes),
        "downloaded file does not match pinned SHA-256"
    );
    let mut tmp = tempfile::NamedTempFile::new_in(directory)?;
    tmp.write_all(&bytes)?;
    tmp.persist(path)?;
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_accepts_only_the_pinned_digest_and_bounded_size() {
        let dir = tempfile::tempdir().unwrap();
        let hash = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
        let path = dir.path().join(hash);
        fs::write(&path, b"abc").unwrap();
        assert_eq!(
            cached_file(dir.path(), "invalid:cache-miss", hash, 3).unwrap(),
            b"abc"
        );
        for invalid in [b"abd".as_slice(), b"", b"abcd"] {
            fs::write(&path, invalid).unwrap();
            assert!(cached_file(dir.path(), "invalid:cache-miss", hash, 3).is_err());
        }
    }
}
