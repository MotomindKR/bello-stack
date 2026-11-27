/// A protobuf enum value without discarding values unknown to this client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnumValue<T> {
    Known(T),
    Unknown(i32),
}

impl<T> EnumValue<T>
where
    T: TryFrom<i32>,
{
    pub(crate) fn from_raw(raw: i32) -> Self {
        T::try_from(raw).map_or(Self::Unknown(raw), Self::Known)
    }

    /// Returns the known value, or `None` when a newer server sent an enum
    /// value this version of the client does not recognize.
    pub fn known(self) -> Option<T> {
        match self {
            Self::Known(value) => Some(value),
            Self::Unknown(_) => None,
        }
    }
}
