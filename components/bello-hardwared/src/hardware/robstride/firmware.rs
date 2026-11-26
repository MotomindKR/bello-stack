use std::{fmt, str::FromStr};

use anyhow::{Context, Result, ensure};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct FirmwareVersion(pub(crate) [u8; 4]);

impl FirmwareVersion {
    pub(crate) fn check_model(self, model: &str) -> Result<()> {
        let family = match model {
            "RS_10P" => Some(10),
            _ => model.strip_prefix("RS_").and_then(|id| id.parse().ok()),
        };
        ensure!(
            self.0[0] == 0 && family == Some(self.0[1]),
            "Firmware {self} does not identify configured model {model}"
        );
        Ok(())
    }
}

impl fmt::Display for FirmwareVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d] = self.0;
        write!(f, "{a}.{b}.{c}.{d}")
    }
}

impl FromStr for FirmwareVersion {
    type Err = anyhow::Error;

    fn from_str(version: &str) -> Result<Self> {
        let mut fields = version.split('.');
        let mut result = [0; 4];
        for field in &mut result {
            *field = fields
                .next()
                .context("Invalid firmware version")?
                .parse()
                .context("Invalid firmware version")?;
        }
        ensure!(fields.next().is_none(), "Invalid firmware version");
        Ok(Self(result))
    }
}
