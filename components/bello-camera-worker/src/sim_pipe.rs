//! Private GStreamer handoff. Caps describe all cameras, including for late joiners.
//! Each buffer contains one protobuf Frame so all clocks travel with its JPEG.
use crate::proto;
use anyhow::{Context, Result, ensure};
use gst::prelude::*;
use prost::Message;
use std::path::Path;

const CAPS: &str = "application/x-bello-camera-frame-v1";

#[derive(Clone, PartialEq, Message)]
pub struct Catalog {
    #[prost(message, repeated, tag = "1")]
    pub streams: Vec<Stream>,
}
#[derive(Clone, PartialEq, Message)]
pub struct Stream {
    #[prost(message, required, tag = "1")]
    pub camera: proto::Camera,
    #[prost(enumeration = "proto::CameraLayout", tag = "2")]
    pub layout: i32,
}

struct Pipeline(gst::Pipeline);
impl Drop for Pipeline {
    fn drop(&mut self) {
        let _ = self.0.set_state(gst::State::Null);
    }
}

impl Pipeline {
    fn start(source: &gst::Element, sink: &gst::Element) -> Result<Self> {
        let pipeline = Self(gst::Pipeline::new());
        pipeline.0.add_many([source, sink])?;
        source.link(sink)?;
        pipeline.0.set_state(gst::State::Playing)?;
        Ok(pipeline)
    }

    fn check(&self) -> Result<()> {
        if let Some(message) = self
            .0
            .bus()
            .unwrap()
            .pop_filtered(&[gst::MessageType::Error, gst::MessageType::Eos])
        {
            match message.view() {
                gst::MessageView::Error(e) => anyhow::bail!("sim camera pipe: {}", e.error()),
                _ => anyhow::bail!("sim camera pipe closed"),
            }
        }
        Ok(())
    }
}

pub struct Output {
    pipeline: Pipeline,
    input: gst_app::AppSrc,
}

impl Output {
    pub async fn open(path: &Path, streams: Vec<Stream>) -> Result<Self> {
        gst::init()?;
        proto::socket::prepare(path).await?;
        let caps = gst::Caps::builder(CAPS)
            .field(
                "catalog",
                gst::Buffer::from_mut_slice(Catalog { streams }.encode_to_vec()),
            )
            .build();
        let input = gst_app::AppSrc::builder()
            .caps(&caps)
            .is_live(true)
            .format(gst::Format::Time)
            .block(true)
            .max_buffers(1)
            .build();
        let sink = gst::ElementFactory::make("unixfdsink")
            .property(
                "socket-path",
                path.to_str().context("non-UTF8 camera socket")?,
            )
            .property("sync", false)
            .property("async", false)
            .build()?;
        let pipeline = Pipeline::start(input.upcast_ref(), &sink)?;
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
        Ok(Self { pipeline, input })
    }
    pub fn publish(&self, frame: proto::Frame) -> Result<()> {
        self.pipeline.check()?;
        ensure!(
            frame.encoded_len() <= proto::MAX_FRAME_MESSAGE_BYTES,
            "sim frame too large"
        );
        self.input
            .push_buffer(gst::Buffer::from_mut_slice(frame.encode_to_vec()))?;
        Ok(())
    }
}

pub struct Input {
    pipeline: Pipeline,
    sink: gst_app::AppSink,
}
impl Input {
    pub fn open(path: &Path) -> Result<Self> {
        gst::init()?;
        let source = gst::ElementFactory::make("unixfdsrc")
            .property(
                "socket-path",
                path.to_str().context("non-UTF8 camera socket")?,
            )
            .build()?;
        let sink = gst_app::AppSink::builder()
            .sync(false)
            .max_buffers(2)
            .build();
        let pipeline = Pipeline::start(&source, sink.upcast_ref())?;
        Ok(Self { pipeline, sink })
    }
    pub fn sample(&self) -> Result<gst::Sample> {
        let sample = self.sink.try_pull_sample(gst::ClockTime::from_seconds(5));
        self.pipeline.check()?;
        sample.context("sim camera pipe stopped producing frames")
    }
    pub fn catalog(sample: &gst::Sample) -> Result<Catalog> {
        let caps = sample
            .caps()
            .and_then(|c| c.structure(0))
            .context("missing camera catalog")?;
        ensure!(caps.name() == CAPS, "unexpected camera pipe format");
        let data = caps.get::<gst::Buffer>("catalog")?;
        ensure!(data.size() <= 1024 * 1024, "oversized camera catalog");
        Ok(Catalog::decode(data.map_readable()?.as_slice())?)
    }
    pub fn frame(sample: &gst::Sample) -> Result<proto::Frame> {
        let data = sample.buffer().context("empty camera sample")?;
        ensure!(
            data.size() <= proto::MAX_FRAME_MESSAGE_BYTES,
            "sim frame too large"
        );
        Ok(proto::Frame::decode(data.map_readable()?.as_slice())?)
    }
}
