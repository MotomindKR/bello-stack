tonic::include_proto!("bello.camera.v1");
pub mod config;
#[cfg(unix)]
pub mod socket;
/// Recommended WatchFrames client receive limit for the largest supported camera.
pub const MAX_FRAME_MESSAGE_BYTES: usize = 32 * 1024 * 1024;

#[cfg(unix)]
pub async fn connect(
    path: &std::path::Path,
) -> Result<tonic::transport::Channel, tonic::transport::Error> {
    let path = path.to_owned();
    tonic::transport::Endpoint::from_static("http://[::]:50000")
        .connect_with_connector(tower::service_fn(move |_| {
            let path = path.clone();
            async move {
                tokio::net::UnixStream::connect(path)
                    .await
                    .map(hyper_util::rt::TokioIo::new)
            }
        }))
        .await
}
