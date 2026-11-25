//! Shared finite-float rule for deployment configuration.

pub fn finite<T: Copy + Into<f64>>(value: &T, _: &()) -> garde::Result {
    if !(*value).into().is_finite() {
        return Err(garde::Error::new("must be finite"));
    }
    Ok(())
}
