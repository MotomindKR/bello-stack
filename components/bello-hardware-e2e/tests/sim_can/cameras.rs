use super::*;
use bello_camera_worker::{monotonic_ns, sim_pipe};
use bello_hardware_api::{
    AcquireLeaseResponse, RenewLeaseRequest,
    camera::{self, camera_source_service_client::CameraSourceServiceClient},
};
use gst::prelude::*;
use std::path::{Path, PathBuf};

const DECLARATION: &str =
    "- {name: stereo, link: base, layout: stereo, calibration: stereo.yaml, fps: 30}";
const CALIBRATION: &str = "width: 16
height: 16
left: &eye {camera_to_link: [0,0,0,1,0,0,0], intrinsics: [8,8,8,8]}
right: *eye
";

pub fn configure(directory: &Path) -> Result<()> {
    use std::io::Write;
    fs::write(directory.join("stereo.yaml"), CALIBRATION)?;
    writeln!(
        fs::OpenOptions::new()
            .append(true)
            .open(directory.join("e2e-hardware.yaml"))?,
        "\ncameras:\n{DECLARATION}"
    )?;
    Ok(())
}
fn fixture_camera(directory: &Path) -> Result<camera::config::Camera> {
    let mut cameras = camera::config::inventory(
        serde_yaml::from_str(DECLARATION)?,
        &directory.join("e2e-hardware.yaml"),
    )?;
    Ok(cameras.remove("stereo").unwrap())
}
struct Source(tokio::task::JoinHandle<()>);
impl Drop for Source {
    fn drop(&mut self) {
        self.0.abort();
    }
}
async fn source(
    socket: &Path,
    calibration: &camera::config::Camera,
    frame: camera::Frame,
) -> Result<Source> {
    let stream = sim_pipe::Stream {
        camera: calibration.info("mujoco"),
        layout: calibration.layout().into(),
    };
    let output = sim_pipe::Output::open(socket, vec![stream]).await?;
    Ok(Source(tokio::spawn(async move {
        loop {
            output.publish(frame.clone()).unwrap();
            sleep(Duration::from_millis(30)).await;
        }
    })))
}
async fn read_both(endpoint: &str, expected: &camera::Frame) -> Result<PathBuf> {
    let mut camera = CameraSourceServiceClient::connect(endpoint.to_owned()).await?;
    let catalog = timeout(Duration::from_secs(15), async {
        loop {
            match camera.get_cameras(camera::GetCamerasRequest {}).await {
                Ok(result) => return anyhow::Ok(result.into_inner()),
                Err(error) if error.code() == tonic::Code::Unavailable => {
                    sleep(Duration::from_millis(30)).await
                }
                Err(error) => return Err(error.into()),
            }
        }
    })
    .await??;
    let metadata: serde_yaml::Value = serde_yaml::from_str(&catalog.cameras[0].metadata_json)?;
    assert_eq!(metadata["stereo"]["left"]["link"].as_str(), Some("base"));
    let local = &catalog.local_sources[&expected.name];
    assert_eq!(local.layout(), camera::CameraLayout::StereoSideBySide);
    let mut frames = camera
        .watch_frames(camera::WatchFramesRequest {
            name: expected.name.clone(),
            latest_only: true,
        })
        .await?
        .into_inner();
    let frame = timeout(Duration::from_secs(3), frames.message())
        .await??
        .context("gRPC camera closed")?
        .frame;
    assert_eq!(frame.as_ref(), Some(expected));
    let reader = gst::ElementFactory::make("unixfdsrc")
        .property("socket-path", &local.unixfd_socket)
        .build()?;
    let sink = gst_app::AppSink::builder().sync(false).build();
    let pipeline = gst::Pipeline::new();
    pipeline.add_many([&reader, sink.upcast_ref()])?;
    reader.link(&sink)?;
    pipeline.set_state(gst::State::Playing)?;
    let sample = sink.try_pull_sample(gst::ClockTime::from_seconds(3));
    pipeline.set_state(gst::State::Null)?;
    let sample = sample.context("missing local JPEG")?;
    assert_eq!(
        sample.buffer().unwrap().map_readable()?.as_slice(),
        expected.jpeg
    );
    Ok(PathBuf::from(&local.unixfd_socket))
}

pub async fn restart_and_check(
    endpoint: &str,
    socket: &Path,
    directory: &Path,
    hardware: &mut HardwareClient,
    lease: &AcquireLeaseResponse,
) -> Result<()> {
    let calibration = fixture_camera(directory)?;
    let now = monotonic_ns();
    let mut frame = camera::Frame {
        name: "stereo".into(),
        sequence: 203,
        capture_ns: now - 100,
        received_ns: now - 5,
        simulation_time_ns: Some(800_000_000),
        jpeg: vec![0xff, 0xd8, 1, 2, 3, 0xff, 0xd9],
        flags: 0,
    };
    let first = source(socket, &calibration, frame.clone()).await?;
    let old = read_both(endpoint, &frame).await?;
    drop(first);
    timeout(Duration::from_secs(12), async {
        while old.exists() {
            sleep(Duration::from_millis(25)).await;
        }
    })
    .await
    .context("failed camera worker did not exit")?;
    hardware
        .renew_lease(RenewLeaseRequest {
            token: lease.token.clone(),
            requested_duration_ms: 30000,
        })
        .await?;
    frame.sequence = 1;
    frame.simulation_time_ns = Some(42);
    let _second = source(socket, &calibration, frame.clone()).await?;
    let new = read_both(endpoint, &frame).await?;
    assert_ne!(old, new);
    Ok(())
}
