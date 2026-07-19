use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceState {
    Starting,
    Connecting,
    Online,
    Degraded,
    Reconnecting,
    Fault,
    Shutdown,
}

impl DeviceState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Starting => "starting",
            Self::Connecting => "connecting",
            Self::Online => "online",
            Self::Degraded => "degraded",
            Self::Reconnecting => "reconnecting",
            Self::Fault => "fault",
            Self::Shutdown => "shutdown",
        }
    }

    pub fn is_online(self) -> bool {
        self == Self::Online
    }
}

impl fmt::Display for DeviceState {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

#[derive(Clone, Debug)]
pub struct DeviceStatus {
    pub state: DeviceState,
    pub detail: String,
    pub transition_count: u64,
}
