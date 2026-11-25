use super::*;
use gst::prelude::*;
use image::{Rgb, RgbImage};
use proto::camera_source_service_server::CameraSourceService;
use tokio_stream::StreamExt;
use tonic::Request;

#[tokio::test]
async fn discovered_local_source_and_watch_frames_deliver_the_same_stereo_jpeg() -> Result<()> {
    let color = |rgb| RgbImage::from_pixel(32, 32, Rgb(rgb));
    let jpeg =
        jpeg::Encoder::new()?.encode(side_by_side(color([250, 0, 0]), color([0, 0, 250])))?;
    let image = image::load_from_memory(&jpeg)?.to_rgb8();
    assert!(image.get_pixel(16, 16)[0] > 240);
    assert!(image.get_pixel(48, 16)[2] > 240);
    let camera = proto::Camera {
        name: "pair".into(),
        width: 64,
        height: 32,
        fps: FPS,
        source: "mujoco".into(),
        ..Default::default()
    };
    let publisher = Publisher::new(vec![(camera, proto::CameraLayout::StereoSideBySide)])?;
    let frames = publisher.frames();
    let catalog = frames
        .get_cameras(Request::new(proto::GetCamerasRequest {}))
        .await?
        .into_inner();
    let source = &catalog.local_sources["pair"];
    assert_eq!(source.layout(), proto::CameraLayout::StereoSideBySide);
    let mut watch = frames
        .watch_frames(Request::new(proto::WatchFramesRequest {
            name: "pair".into(),
            latest_only: true,
        }))
        .await?
        .into_inner();
    let publish = |sequence| {
        let now = monotonic_ns();
        publisher.publish(proto::Frame {
            name: "pair".into(),
            sequence,
            jpeg: jpeg.clone(),
            capture_ns: now,
            received_ns: now,
            simulation_time_ns: Some(sequence),
            ..Default::default()
        })
    };
    publish(1)?;
    let receiver = gst::parse::launch(&format!(
        "unixfdsrc socket-path={} ! appsink name=out sync=false",
        source.unixfd_socket
    ))?
    .downcast::<gst::Pipeline>()
    .unwrap();
    let out = receiver
        .by_name("out")
        .unwrap()
        .downcast::<gst_app::AppSink>()
        .unwrap();
    receiver.set_state(gst::State::Playing)?;
    let mut sample = None;
    for seq in 2..102 {
        publish(seq)?;
        sample = out.try_pull_sample(gst::ClockTime::from_mseconds(30));
        if sample.is_some() {
            break;
        }
    }
    receiver.set_state(gst::State::Null)?;
    let sample = sample.context("no JPEG on discovered socket")?;
    let grpc = watch.next().await.unwrap()?.frame.unwrap();
    assert_eq!(
        sample.buffer().unwrap().map_readable()?.as_slice(),
        grpc.jpeg
    );
    Ok(())
}
