//! Publish one encoded exposure to WatchFrames and local GStreamer consumers.
use crate::{Frames, proto};
use anyhow::{Context, Result, ensure};
use gst::prelude::*;
use std::{collections::HashMap, os::unix::fs::PermissionsExt, sync::Arc};

pub struct Publisher {
    frames: Frames,
    streams: HashMap<String, Stream>,
    _sockets: tempfile::TempDir,
}

struct Stream {
    pipeline: gst::Pipeline,
    input: gst_app::AppSrc,
}

impl Drop for Stream {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

struct Jpeg(Arc<proto::Frame>);

impl AsRef<[u8]> for Jpeg {
    fn as_ref(&self) -> &[u8] {
        &self.0.jpeg
    }
}

impl Publisher {
    pub fn new(cameras: Vec<(proto::Camera, proto::CameraLayout)>) -> Result<Self> {
        gst::init()?;
        let mut frames = Frames::configured(cameras.iter().map(|(c, _)| c.clone()).collect())?;
        let sockets = tempfile::Builder::new()
            .permissions(std::fs::Permissions::from_mode(0o700))
            .tempdir_in("/tmp")?;
        let mut streams = HashMap::new();
        for (i, (camera, layout)) in cameras.into_iter().enumerate() {
            ensure!(
                matches!(
                    layout,
                    proto::CameraLayout::Mono | proto::CameraLayout::StereoSideBySide
                ),
                "missing camera layout"
            );
            ensure!(
                layout != proto::CameraLayout::StereoSideBySide || camera.width.is_multiple_of(2),
                "stereo requires equal eye widths"
            );
            let path = sockets.path().join(format!("{i}.sock"));
            let caps = gst::Caps::builder("image/jpeg")
                .field("width", camera.width as i32)
                .field("height", camera.height as i32)
                .field("framerate", gst::Fraction::new(camera.fps as i32, 1))
                .build();
            let input = gst_app::AppSrc::builder()
                .caps(&caps)
                .is_live(true)
                .format(gst::Format::Time)
                .max_buffers(1)
                .leaky_type(gst_app::AppLeakyType::Downstream)
                .build();
            let output = gst::ElementFactory::make("unixfdsink")
                .property("socket-path", path.to_str().unwrap())
                .property("sync", false)
                .property("async", false)
                .build()
                .context("install the GStreamer unixfd plugin")?;
            ensure!(
                output.has_property("min-memory-size"),
                "unixfdsink requires GStreamer 1.28 or later to share JPEG buffers"
            );
            let pipeline = gst::Pipeline::new();
            pipeline.add_many([input.upcast_ref::<gst::Element>(), &output])?;
            input.link(&output)?;
            let stream = Stream { pipeline, input };
            stream.pipeline.set_state(gst::State::Playing)?;
            Arc::make_mut(&mut frames.local_sources).insert(
                camera.name.clone(),
                proto::LocalCameraSource {
                    layout: layout.into(),
                    unixfd_socket: path.to_str().unwrap().to_owned(),
                },
            );
            streams.insert(camera.name, stream);
        }
        Ok(Self {
            frames,
            streams,
            _sockets: sockets,
        })
    }

    pub fn frames(&self) -> &Frames {
        &self.frames
    }

    pub fn publish(&self, frame: proto::Frame) -> Result<()> {
        let stream = self.streams.get(&frame.name).context("unknown camera")?;
        if let Some(message) = stream
            .pipeline
            .bus()
            .unwrap()
            .pop_filtered(&[gst::MessageType::Error])
            && let gst::MessageView::Error(error) = message.view()
        {
            anyhow::bail!("local camera stream failed: {}", error.error());
        }
        let frame = Arc::new(frame);
        self.frames.publish_shared(frame.clone())?;
        let capture_age = crate::monotonic_ns().saturating_sub(frame.capture_ns);
        let pts = stream
            .pipeline
            .current_running_time()
            .unwrap_or_default()
            .saturating_sub(gst::ClockTime::from_nseconds(capture_age));
        let mut buffer = gst::Buffer::from_slice(Jpeg(frame));
        buffer.get_mut().unwrap().set_pts(pts);
        stream.input.push_buffer(buffer)?;
        Ok(())
    }
}
