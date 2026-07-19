use std::time::Duration;

use crate::Error;

pub(crate) fn positive_milliseconds(value: Duration, field: &str) -> Result<u64, Error> {
    if value.is_zero() {
        return Err(Error::InvalidArgument(format!("{field} must be positive")));
    }
    let milliseconds = value.as_nanos().div_ceil(1_000_000);
    u64::try_from(milliseconds).map_err(|_| Error::InvalidArgument(format!("{field} is too large")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn positive_sub_millisecond_values_round_up() {
        assert_eq!(
            positive_milliseconds(Duration::from_nanos(1), "duration").unwrap(),
            1
        );
        assert_eq!(
            positive_milliseconds(Duration::from_millis(2), "duration").unwrap(),
            2
        );
    }

    #[test]
    fn zero_is_not_confused_with_the_wire_default_sentinel() {
        assert!(positive_milliseconds(Duration::ZERO, "duration").is_err());
    }
}
