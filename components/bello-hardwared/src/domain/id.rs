use std::fmt;
use std::str::FromStr;
use std::sync::Arc;

use thiserror::Error;
use uuid::Uuid;

macro_rules! string_id {
    ($name:ident) => {
        #[derive(Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
        pub struct $name(Arc<str>);

        impl $name {
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl From<String> for $name {
            fn from(value: String) -> Self {
                Self(value.into())
            }
        }

        impl From<&str> for $name {
            fn from(value: &str) -> Self {
                Self(value.into())
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str(self.as_str())
            }
        }
    };
}

string_id!(BusName);
string_id!(GroupName);
string_id!(ImuName);
string_id!(PowerBoardName);
string_id!(MotorName);
string_id!(SerialDeviceName);

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct LeaseToken(Uuid);

impl LeaseToken {
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for LeaseToken {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for LeaseToken {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

impl FromStr for LeaseToken {
    type Err = InvalidLeaseToken;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Uuid::parse_str(value)
            .map(Self)
            .map_err(|_| InvalidLeaseToken)
    }
}

#[derive(Clone, Copy, Debug, Error)]
#[error("invalid lease token")]
pub struct InvalidLeaseToken;

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
pub struct MotorAddress {
    pub bus: BusName,
    pub id: u8,
}
