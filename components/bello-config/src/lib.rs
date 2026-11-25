//! Field rules shared by the deployment config readers.

pub fn trimmed_name(value: &str, _: &()) -> garde::Result {
    if value.is_empty() || value.trim() != value {
        return Err(garde::Error::new(
            "must be nonempty without surrounding whitespace",
        ));
    }
    Ok(())
}

pub fn finite<T: Copy + Into<f64>>(value: &T, _: &()) -> garde::Result {
    if !(*value).into().is_finite() {
        return Err(garde::Error::new("must be finite"));
    }
    Ok(())
}

pub fn positive(value: &f64, _: &()) -> garde::Result {
    if !value.is_finite() || *value <= 0.0 {
        return Err(garde::Error::new("must be finite and positive"));
    }
    Ok(())
}

pub fn nonnegative(value: &f64, _: &()) -> garde::Result {
    if !value.is_finite() || *value < 0.0 {
        return Err(garde::Error::new("must be finite and nonnegative"));
    }
    Ok(())
}
