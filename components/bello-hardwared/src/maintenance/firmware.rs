use std::borrow::Cow;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, ensure};

use super::{Link, Version};

const RELEASES: &[(&str, &str, &[u8])] = include!(concat!(env!("OUT_DIR"), "/firmware.rs"));
const ACK_TIMEOUT: Duration = Duration::from_secs(2);
const ACK_ATTEMPTS: usize = 3;
const MAX_RESUMES: usize = 30;
const START: u8 = 11;
const INFO: u8 = 12;
const DATA: u8 = 13;
const END: u8 = 14;

pub(super) struct Image {
    pub(super) version: Version,
    bytes: Cow<'static, [u8]>,
}

impl Image {
    pub(super) fn prepare(model: &str, installed: Version, simulation: bool) -> Result<Self> {
        if simulation {
            let version = Version([0, installed.0[1], 0, 2]);
            require_upgrade(installed, version)?;
            return Ok(Self {
                version,
                bytes: format!("BELLO-SIM-FW/1\n{version}").into_bytes().into(),
            });
        }

        let &(_, version, bytes) = RELEASES
            .iter()
            .find(|(name, _, _)| *name == model)
            .with_context(|| format!("No embedded firmware release for {model}"))?;
        let version: Version = version.parse()?;
        version.check_model(model)?;
        require_upgrade(installed, version)?;
        Ok(Self {
            version,
            bytes: Cow::Borrowed(bytes),
        })
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
