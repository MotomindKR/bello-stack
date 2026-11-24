use std::io;

#[cfg(unix)]
pub(super) fn monotonic_ns() -> io::Result<u64> {
    #[cfg(target_os = "macos")]
    let clock = libc::CLOCK_UPTIME_RAW;
    #[cfg(not(target_os = "macos"))]
    let clock = libc::CLOCK_MONOTONIC;
    let mut time = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: time points to writable storage for a timespec.
    if unsafe { libc::clock_gettime(clock, &mut time) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(time.tv_sec as u64 * 1_000_000_000 + time.tv_nsec as u64)
}

#[cfg(windows)]
pub(super) fn monotonic_ns() -> io::Result<u64> {
    use std::sync::OnceLock;
    use windows_sys::Win32::System::Performance::{
        QueryPerformanceCounter, QueryPerformanceFrequency,
    };

    static FREQUENCY: OnceLock<u64> = OnceLock::new();
    let frequency = *FREQUENCY.get_or_init(|| {
        let mut frequency = 0;
        // SAFETY: frequency points to writable storage for a LARGE_INTEGER.
        if unsafe { QueryPerformanceFrequency(&mut frequency) } == 0 {
            return 0;
        }
        frequency as u64
    });
    if frequency == 0 {
        return Err(io::Error::other("Windows performance counter unavailable"));
    }
    let mut ticks = 0;
    // SAFETY: ticks points to writable storage for a LARGE_INTEGER.
    if unsafe { QueryPerformanceCounter(&mut ticks) } == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok((ticks as u128 * 1_000_000_000 / u128::from(frequency)) as u64)
}
