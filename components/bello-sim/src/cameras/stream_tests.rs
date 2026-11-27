use super::*;
use gst::prelude::*;
use image::{Rgb, RgbImage};
use proto::camera_source_service_client::CameraSourceServiceClient;
use proto::camera_source_service_server::CameraSourceServiceServer;
use rand::{RngExt, SeedableRng, rngs::SmallRng};
use tokio_stream::wrappers::UnixListenerStream;
use tonic::Request;

#[tokio::test]
async fn discovered_local_source_and_watch_frames_deliver_the_same_stereo_jpeg() -> Result<()> {
    check_stereo_stream(1600, 1200, 2 * 1024 * 1024).await?;
    check_stereo_stream(2048, 2160, 4 * 1024 * 1024).await
}

async fn check_stereo_stream(eye_width: u32, height: u32, minimum_bytes: usize) -> Result<()> {
    let mut rng = SmallRng::seed_from_u64(23);
    let mut eye = |marker| {
        RgbImage::from_fn(eye_width, height, |x, y| {
            if x < 32 && y < 32 {
                Rgb(marker)
            } else {
                Rgb([rng.random(); 3])
            }
        })
    };
    let jpeg = jpeg::Encoder::new()?.encode(side_by_side(eye([250, 0, 0]), eye([0, 0, 250])))?;
    assert!(jpeg.len() > minimum_bytes);
    let image = image::load_from_memory(&jpeg)?.to_rgb8();
    assert!(image.get_pixel(16, 16)[0] > 240);
    assert!(image.get_pixel(eye_width + 16, 16)[2] > 240);
    let camera = proto::Camera {
        name: "pair".into(),
        width: eye_width * 2,
        height,
        fps: FPS,
        source: "mujoco".into(),
        ..Default::default()
    };
    let publisher = Publisher::new(vec![(camera, proto::CameraLayout::StereoSideBySide)])?;
    let temp = tempfile::tempdir()?;
    let path = temp.path().join("frames.sock");
    let (listener, _socket) = bello_camera_api::socket::bind(&path).await?;
    let server = tokio::spawn(
        tonic::transport::Server::builder()
            .add_service(CameraSourceServiceServer::new(publisher.frames().clone()))
            .serve_with_incoming(UnixListenerStream::new(listener)),
    );
    let mut client = CameraSourceServiceClient::new(bello_camera_api::connect(&path).await?);
    if jpeg.len() > 4 * 1024 * 1024 {
        client = client.max_decoding_message_size(bello_camera_api::MAX_FRAME_MESSAGE_BYTES);
    }
    let catalog = client
        .get_cameras(Request::new(proto::GetCamerasRequest {}))
        .await?
        .into_inner();
    let source = &catalog.local_sources["pair"];
    assert_eq!(source.layout(), proto::CameraLayout::StereoSideBySide);
    let mut watch = client
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
    let grpc = watch.message().await?.unwrap().frame.unwrap();
    server.abort();
    assert_eq!(
        sample.buffer().unwrap().map_readable()?.as_slice(),
        grpc.jpeg
    );
    Ok(())
}
