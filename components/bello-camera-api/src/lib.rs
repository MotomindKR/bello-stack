//! Timestamped camera frames shared by hardware capture, HIL rendering and recording.
use std::{pin::Pin, sync::Arc};
use tokio::sync::{broadcast, watch};
use tonic::{Request, Response, Status};
pub mod proto {
    #![allow(clippy::double_must_use)]
    tonic::include_proto!("bello.camera.v1");
}
pub mod socket;
pub const FILE_DESCRIPTOR_SET: &[u8] =
    tonic::include_file_descriptor_set!("bello_camera_v1_descriptor");
pub const NAMES: [&str; 3] = ["birds_eye", "right_gripper", "left_gripper"];
pub const WIDTH: u32 = 640;
pub const HEIGHT: u32 = 360;
pub const FPS: u32 = 30;

pub fn monotonic_ns() -> u64 {
    let mut time = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // CLOCK_MONOTONIC is common to all local source, recorder and controller processes.
    assert_eq!(
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut time) },
        0
    );
    time.tv_sec as u64 * 1_000_000_000 + time.tv_nsec as u64
}

struct Slot {
    ordered: broadcast::Sender<Arc<proto::Frame>>,
    latest: watch::Sender<Option<Arc<proto::Frame>>>,
}
#[derive(Clone)]
pub struct Frames {
    cameras: Arc<Vec<proto::Camera>>,
    slots: Arc<Vec<Slot>>,
}
impl Frames {
    pub fn new(cameras: Vec<proto::Camera>) -> Result<Self, Status> {
        if cameras.len() != 3
            || NAMES
                .iter()
                .any(|n| cameras.iter().filter(|c| c.name == *n).count() != 1)
            || cameras.iter().any(|c| {
                c.width != WIDTH
                    || c.height != HEIGHT
                    || c.fps != FPS
                    || !matches!(c.source.as_str(), "v4l2" | "mujoco")
            })
        {
            return Err(Status::invalid_argument(
                "requires three named 640x360, 30 Hz cameras",
            ));
        }
        let slots = cameras
            .iter()
            .map(|_| Slot {
                ordered: broadcast::channel(8).0,
                latest: watch::channel(None).0,
            })
            .collect();
        Ok(Self {
            cameras: Arc::new(cameras),
            slots: Arc::new(slots),
        })
    }
    pub fn cameras(&self) -> &[proto::Camera] {
        &self.cameras
    }
    pub fn publish(&self, frame: proto::Frame) -> Result<(), Status> {
        let i = self
            .cameras
            .iter()
            .position(|c| c.name == frame.name)
            .ok_or_else(|| Status::invalid_argument("unknown camera"))?;
        if (self.cameras[i].source == "mujoco") != frame.simulation_time_ns.is_some() {
            return Err(Status::invalid_argument(
                "frame clock does not match its declared source",
            ));
        }
        if frame.jpeg.len() > 2 * 1024 * 1024
            || !frame.jpeg.starts_with(&[0xff, 0xd8])
            || !frame.jpeg.ends_with(&[0xff, 0xd9])
            || frame.capture_ns == 0
            || frame.received_ns < frame.capture_ns
        {
            return Err(Status::invalid_argument("invalid camera frame"));
        }
        let frame = Arc::new(frame);
        self.slots[i].latest.send_replace(Some(frame.clone()));
        let _ = self.slots[i].ordered.send(frame);
        Ok(())
    }
}

#[tonic::async_trait]
impl proto::camera_source_service_server::CameraSourceService for Frames {
    async fn get_cameras(
        &self,
        _: Request<proto::GetCamerasRequest>,
    ) -> Result<Response<proto::GetCamerasResponse>, Status> {
        Ok(Response::new(proto::GetCamerasResponse {
            cameras: self.cameras.as_ref().clone(),
        }))
    }
    type WatchFramesStream = Pin<
        Box<dyn tokio_stream::Stream<Item = Result<proto::WatchFramesResponse, Status>> + Send>,
    >;
    async fn watch_frames(
        &self,
        request: Request<proto::WatchFramesRequest>,
    ) -> Result<Response<Self::WatchFramesStream>, Status> {
        let r = request.into_inner();
        let index = self
            .cameras
            .iter()
            .position(|c| c.name == r.name)
            .ok_or_else(|| Status::not_found("unknown camera"))?;
        let stream: Self::WatchFramesStream = if r.latest_only {
            let mut receiver = self.slots[index].latest.subscribe();
            Box::pin(async_stream::try_stream! {
                loop {
                    let frame = receiver.borrow_and_update().clone();
                    if let Some(frame) = frame { yield proto::WatchFramesResponse {frame: Some(frame.as_ref().clone())}; }
                    if receiver.changed().await.is_err() { break; }
                }
            })
        } else {
            let mut receiver = self.slots[index].ordered.subscribe();
            Box::pin(async_stream::try_stream! {
                loop { match receiver.recv().await {
                    Ok(frame) => yield proto::WatchFramesResponse {frame: Some(frame.as_ref().clone())},
                    Err(broadcast::error::RecvError::Closed) => break,
                    Err(broadcast::error::RecvError::Lagged(_)) => Err(Status::resource_exhausted("camera consumer missed frames"))?,
                }}
            })
        };
        Ok(Response::new(stream))
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use proto::camera_source_service_server::CameraSourceService;
    use tokio_stream::StreamExt;
    fn frames() -> Frames {
        Frames::new(
            NAMES
                .iter()
                .map(|n| proto::Camera {
                    name: (*n).into(),
                    width: WIDTH,
                    height: HEIGHT,
                    fps: FPS,
                    source: "mujoco".into(),
                    metadata_json: "{}".into(),
                })
                .collect(),
        )
        .unwrap()
    }
    fn frame(sequence: u64) -> proto::Frame {
        proto::Frame {
            name: NAMES[0].into(),
            sequence,
            capture_ns: sequence + 1,
            received_ns: sequence + 1,
            jpeg: vec![0xff, 0xd8, 0xff, 0xd9],
            flags: 0,
            simulation_time_ns: Some(sequence),
        }
    }
    #[tokio::test]
    async fn preview_skips_old_frames_and_recording_reports_lag() {
        let source = frames();
        let mut preview = source
            .watch_frames(Request::new(proto::WatchFramesRequest {
                name: NAMES[0].into(),
                latest_only: true,
            }))
            .await
            .unwrap()
            .into_inner();
        let mut recording = source
            .watch_frames(Request::new(proto::WatchFramesRequest {
                name: NAMES[0].into(),
                latest_only: false,
            }))
            .await
            .unwrap()
            .into_inner();
        for seq in 0..20 {
            source.publish(frame(seq)).unwrap();
        }
        assert_eq!(
            preview
                .next()
                .await
                .unwrap()
                .unwrap()
                .frame
                .unwrap()
                .sequence,
            19
        );
        assert_eq!(
            recording.next().await.unwrap().unwrap_err().code(),
            tonic::Code::ResourceExhausted
        );
    }
    #[test]
    fn invalid_source_clock_and_camera_mapping_are_rejected() {
        let source = frames();
        let mut value = frame(1);
        value.simulation_time_ns = None;
        assert!(source.publish(value).is_err());
        let mut cameras = source.cameras().to_vec();
        cameras[1].name = NAMES[0].into();
        assert!(Frames::new(cameras).is_err());
    }
}
