use anyhow::{Context, Result, ensure};
use bello_camera_worker::{media::Publisher, proto, sim_pipe::Input};
use proto::camera_source_service_server::CameraSourceServiceServer;
use proto::config::{Binding, Camera, Source, Transport};
use std::collections::{BTreeMap, BTreeSet};
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

fn sim_streams(
    input: Input,
    expected: Vec<Camera>,
    output: Arc<Publisher>,
    failed: mpsc::UnboundedSender<Result<()>>,
) -> Result<()> {
    let first = input.sample()?;
    let catalog = Input::catalog(&first)?;
    let mut names = BTreeSet::new();
    for camera in expected {
        let stream = catalog
            .streams
            .iter()
            .find(|s| s.camera.name == camera.name)
            .with_context(|| format!("simulator is missing camera {}", camera.name))?;
        ensure!(
            stream.layout() == camera.layout() && stream.camera == camera.info("mujoco"),
            "simulator camera {} disagrees with the hardware description",
            camera.name
        );
        names.insert(camera.name);
    }
    std::thread::spawn(move || {
        let result = (|| {
            let mut sample = first;
            loop {
                let frame = Input::frame(&sample)?;
                if names.contains(&frame.name) {
                    output.publish(frame)?;
                }
                sample = input.sample()?;
            }
        })();
        let _ = failed.send(result);
    });
    Ok(())
}

async fn serve(source: Source, directory: PathBuf) -> Result<()> {
    let (failed, mut failure) = mpsc::unbounded_channel();
    let cameras = source.0.iter().map(Binding::stream_info).collect();
    let publisher = Arc::new(Publisher::new_in(cameras, &directory)?);
    let mut sockets = BTreeMap::<_, Vec<Camera>>::new();
    for binding in source.0 {
        match binding.transport {
            Transport::Mujoco { socket } => sockets.entry(socket).or_default().push(binding.camera),
            Transport::V4l2 { device } => {
                #[cfg(target_os = "linux")]
                {
                    let output = publisher.clone();
                    let failed = failed.clone();
                    std::thread::spawn(move || {
                        let _ = failed.send(usb::capture(binding.camera, &device, &output));
                    });
                }
                #[cfg(not(target_os = "linux"))]
                anyhow::bail!("USB camera {} requires Linux V4L2", device.display());
            }
        }
    }
    for (path, cameras) in sockets {
        sim_streams(
            Input::open(&path)?,
            cameras,
            publisher.clone(),
            failed.clone(),
        )?;
    }
    drop(failed);
    let (listener, _owner) = proto::socket::bind(&directory.join("rpc")).await?;
    let server = tonic::transport::Server::builder()
        .add_service(CameraSourceServiceServer::new(publisher.frames().clone()))
        .serve_with_incoming(UnixListenerStream::new(listener));
    tokio::select! {
        result = server => Ok(result?),
        result = failure.recv() => result.context("camera capture stopped")?,
    }
}
