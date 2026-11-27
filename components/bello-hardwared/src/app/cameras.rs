#[cfg(unix)]
use bello_hardware_api::camera;
use bello_hardware_api::camera::{
    GetCamerasRequest, GetCamerasResponse, WatchFramesRequest, WatchFramesResponse,
    camera_source_service_client::CameraSourceServiceClient,
    camera_source_service_server::CameraSourceService, config::Source,
};
use std::{path::PathBuf, pin::Pin};
use tokio::sync::watch;
use tonic::{Request, Response, Status, transport::Channel};

#[derive(Clone)]
pub(super) struct Cameras {
    worker: Option<watch::Receiver<Option<PathBuf>>>,
}

impl Cameras {
    pub(super) fn start(source: Option<Source>, shutdown: watch::Receiver<bool>) -> Self {
        let worker = source.map(|source| {
            let (address, receiver) = watch::channel(None);
            #[cfg(unix)]
            tokio::spawn(supervise(source, address, shutdown));
            #[cfg(not(unix))]
            let _ = (source, address, shutdown);
            receiver
        });
        Self { worker }
    }

    async fn client(&self) -> Result<CameraSourceServiceClient<Channel>, Status> {
        #[cfg(unix)]
        if let Some(socket) = self
            .worker
            .as_ref()
            .and_then(|worker| worker.borrow().clone())
        {
            let channel = camera::connect(&socket)
                .await
                .map_err(|_| Status::unavailable("camera worker is starting or offline"))?;
            return Ok(CameraSourceServiceClient::new(channel)
                .max_decoding_message_size(camera::MAX_FRAME_MESSAGE_BYTES));
        }
        Err(Status::unavailable("camera worker is offline"))
    }
}

#[tonic::async_trait]
impl CameraSourceService for Cameras {
    async fn get_cameras(
        &self,
        request: Request<GetCamerasRequest>,
    ) -> Result<Response<GetCamerasResponse>, Status> {
        if self.worker.is_none() {
            return Ok(Response::new(GetCamerasResponse::default()));
        }
        self.client().await?.get_cameras(request.into_inner()).await
    }

    type WatchFramesStream =
        Pin<Box<dyn futures_core::Stream<Item = Result<WatchFramesResponse, Status>> + Send>>;
    async fn watch_frames(
        &self,
        request: Request<WatchFramesRequest>,
    ) -> Result<Response<Self::WatchFramesStream>, Status> {
        if self.worker.is_none() {
            return Err(Status::not_found("no cameras configured"));
        }
        let stream = self
            .client()
            .await?
            .watch_frames(request.into_inner())
            .await?
            .into_inner();
        Ok(Response::new(Box::pin(stream)))
    }
}

#[cfg(unix)]
async fn supervise(
    source: Source,
    address: watch::Sender<Option<PathBuf>>,
    mut shutdown: watch::Receiver<bool>,
) {
    use std::time::Duration;
    while !*shutdown.borrow() {
        let result = run_worker(&source, &address, &mut shutdown).await;
        address.send_replace(None);
        if *shutdown.borrow() || shutdown.has_changed().is_err() {
            break;
        }
        if let Err(error) = result {
            tracing::warn!(%error, "camera worker unavailable; retrying");
        }
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_secs(2)) => {},
            _ = shutdown.changed() => break,
        }
    }
}

#[cfg(unix)]
async fn run_worker(
    source: &Source,
    address: &watch::Sender<Option<PathBuf>>,
    shutdown: &mut watch::Receiver<bool>,
) -> anyhow::Result<()> {
    use anyhow::{Context, ensure};
    use std::{os::unix::fs::PermissionsExt, process::Stdio};
    use tokio::io::AsyncWriteExt;
    let directory = tempfile::Builder::new()
        .prefix("bello-cam-")
        .permissions(std::fs::Permissions::from_mode(0o700))
        .tempdir_in("/tmp")?;
    let executable = std::env::current_exe()?.with_file_name("bello-camera-worker");
    let mut child = tokio::process::Command::new(&executable)
        .arg(directory.path())
        .stdin(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .with_context(|| format!("launch {}", executable.display()))?;
    let config = serde_json::to_vec(source)?;
    let mut stdin = child.stdin.take().context("missing worker stdin")?;
    stdin.write_u32(u32::try_from(config.len())?).await?;
    stdin.write_all(&config).await?;
    address.send_replace(Some(directory.path().join("rpc")));
    tokio::select! {
        result = child.wait() => ensure!(result?.success(), "camera worker exited with an error"),
        _ = shutdown.changed() => {
            child.kill().await?;
        }
    }
    Ok(())
}
