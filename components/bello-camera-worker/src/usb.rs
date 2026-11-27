use anyhow::{Context, Result, ensure};
use bello_camera_worker::{media::Publisher, monotonic_ns, proto, proto::config::Device};
use std::time::Duration;
use v4l::{
    Device as Camera, Format, FourCC,
    buffer::{Flags, Metadata, Type},
    io::traits::CaptureStream,
    prelude::MmapStream,
    video::{Capture, capture::Parameters},
};

fn configure(source: &Device) -> Result<Camera> {
    let camera = Camera::with_path(&source.device)
        .with_context(|| format!("open camera {}", source.device.display()))?;
    let mjpeg = FourCC::new(b"MJPG");
    let format = camera.set_format(&Format::new(source.width, source.height, mjpeg))?;
    ensure!(
        (format.width, format.height, format.fourcc) == (source.width, source.height, mjpeg),
        "{} negotiated unsupported format {format}",
        source.name
    );
    let interval = camera
        .set_params(&Parameters::with_fps(source.fps))?
        .interval;
    ensure!(
        interval.numerator != 0
            && interval.numerator.checked_mul(source.fps) == Some(interval.denominator),
        "{} negotiated unsupported frame interval {interval}",
        source.name
    );
    Ok(camera)
}

fn capture_time(meta: &Metadata) -> Result<u64> {
    ensure!(
        meta.flags & Flags::TIMESTAMP_MASK == Flags::TIMESTAMP_MONOTONIC,
        "camera supplied a timestamp outside the host monotonic clock"
    );
    let ts = meta.timestamp;
    ensure!(
        ts.sec >= 0 && (0..1_000_000).contains(&ts.usec),
        "invalid capture timestamp {ts}"
    );
    Ok(Duration::from(ts).as_nanos().try_into()?)
}

pub fn capture(source: Device, output: &Publisher) -> Result<()> {
    let camera = configure(&source)?;
    let mut buffers = MmapStream::with_buffers(&camera, Type::VideoCapture, 4)?;
    buffers.set_timeout(Duration::from_secs(2));
    loop {
        let (buffer, meta) = buffers
            .next()
            .with_context(|| format!("read camera {}", source.name))?;
        let received_ns = monotonic_ns();
        if meta.flags.contains(Flags::ERROR) {
            continue;
        }
        let image = buffer
            .get(..meta.bytesused as usize)
            .context("truncated capture buffer")?;
        output.publish(proto::Frame {
            capture_ns: capture_time(meta)?,
            received_ns,
            name: source.name.clone(),
            jpeg: image.to_vec(),
            flags: meta.flags.bits(),
            sequence: meta.sequence.into(),
            simulation_time_ns: None,
        })?;
    }
}
