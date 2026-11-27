use anyhow::{Context, Result, ensure};
use bello_camera_worker::{media::Publisher, proto, sim_pipe::Input};
use proto::camera_source_service_server::CameraSourceServiceServer;
use proto::config::{Device, Source};
use std::{io::Read, path::PathBuf, sync::Arc};
use tokio::sync::mpsc;
use tokio_stream::wrappers::UnixListenerStream;

#[cfg(target_os = "linux")]
mod usb;

// hardwared owns stdin. EOF also terminates capture if the supervisor is killed.
fn main() -> Result<()> {
    let mut input = std::io::stdin().lock();
    let mut length = [0; 4];
    input.read_exact(&mut length)?;
    let length = u32::from_be_bytes(length) as usize;
    ensure!(length <= 1024 * 1024, "oversized camera configuration");
    let mut bytes = vec![0; length];
    input.read_exact(&mut bytes)?;
    drop(input);
    let source: Source = serde_json::from_slice(&bytes)?;
    source.validate().map_err(anyhow::Error::msg)?;
    std::thread::spawn(|| {
        let _ = std::io::stdin().read(&mut [0]);
        std::process::exit(0);
    });
    let directory = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .context("missing worker directory")?,
    );
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    runtime.block_on(serve(source, directory))
}

fn device_info(d: &Device) -> (proto::Camera, proto::CameraLayout) {
    (
        proto::Camera {
            name: d.name.clone(),
            width: d.width,
            height: d.height,
            fps: d.fps,
            source: "v4l2".into(),
            metadata_json: String::new(),
        },
        if d.stereo {
            proto::CameraLayout::StereoSideBySide
        } else {
            proto::CameraLayout::Mono
        },
    )
}

async fn serve(source: Source, directory: PathBuf) -> Result<()> {
    let (failed, mut failure) = mpsc::unbounded_channel();
    let publisher = match source {
        Source::Mujoco { socket } => {
            let input = Input::open(&socket)?;
            let first = input.sample()?;
            let catalog = Input::catalog(&first)?;
            let cameras = catalog
                .streams
                .into_iter()
                .map(|s| (s.camera.clone(), s.layout()))
                .collect();
            let publisher = Arc::new(Publisher::new_in(cameras, &directory)?);
            let output = publisher.clone();
            std::thread::spawn(move || {
                let result: Result<()> = (|| {
                    let mut sample = first;
                    loop {
                        output.publish(Input::frame(&sample)?)?;
                        sample = input.sample()?;
                    }
                })();
                let _ = failed.send(result);
            });
            publisher
        }
        Source::V4l2 { devices } => {
            let publisher = Arc::new(Publisher::new_in(
                devices.iter().map(device_info).collect(),
                &directory,
            )?);
            #[cfg(target_os = "linux")]
            for device in devices {
                let output = publisher.clone();
                let failed = failed.clone();
                std::thread::spawn(move || {
                    let _ = failed.send(usb::capture(device, &output));
                });
            }
            #[cfg(not(target_os = "linux"))]
            let _ = failed.send(Err(anyhow::anyhow!(
                "USB camera capture requires Linux V4L2"
            )));
            publisher
        }
    };
    let (listener, _owner) = proto::socket::bind(&directory.join("rpc")).await?;
    let server = tonic::transport::Server::builder()
        .add_service(CameraSourceServiceServer::new(publisher.frames().clone()))
        .serve_with_incoming(UnixListenerStream::new(listener));
    tokio::select! {
        result = server => Ok(result?),
        result = failure.recv() => result.context("camera capture stopped")?,
    }
}
