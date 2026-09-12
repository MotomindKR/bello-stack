//! Latest-sample pose streaming with bounded storage and cancellation.
use crate::BodyState;
use bello_pose_api::{
    pose_service_server::{PoseService, PoseServiceServer},
    *,
};
use futures_core::Stream;
use std::pin::Pin;
use std::time::Instant;
use tokio::sync::watch;
use tonic::{Request, Response, Status};

#[derive(Clone)]
pub struct Sample {
    pub message: WatchPoseResponse,
    pub captured: Instant,
}

pub fn sample(
    body: BodyState,
    reference: &str,
    tracked: &str,
    sequence: u64,
    epoch: Instant,
) -> Sample {
    let [x, y, z] = body.position;
    let [w, qx, qy, qz] = body.orientation_wxyz;
    let [vx, vy, vz] = body.linear_velocity_world;
    let [wx, wy, wz] = body.angular_velocity_local;
    let captured = Instant::now();
    Sample {
        captured,
        message: WatchPoseResponse {
            reference_frame: reference.into(),
            tracked_frame: tracked.into(),
            sequence,
            translation_metres: Some(Translation { x, y, z }),
            orientation_xyzw: Some(Orientation {
                x: qx,
                y: qy,
                z: qz,
                w,
            }),
            linear_velocity_metres_per_second: Some(LinearVelocity {
                x: vx,
                y: vy,
                z: vz,
            }),
            angular_velocity_radians_per_second: Some(AngularVelocity {
                x: wx,
                y: wy,
                z: wz,
            }),
            sample_age_ms: 0,
            sample_monotonic_ns: captured.duration_since(epoch).as_nanos() as u64,
        },
    }
}

pub struct Service {
    latest: watch::Receiver<Sample>,
    stop: watch::Receiver<bool>,
}

impl Service {
    pub fn new(
        latest: watch::Receiver<Sample>,
        stop: watch::Receiver<bool>,
    ) -> PoseServiceServer<Self> {
        PoseServiceServer::new(Self { latest, stop })
    }
}

#[tonic::async_trait]
impl PoseService for Service {
    type WatchPoseStream = Pin<Box<dyn Stream<Item = Result<WatchPoseResponse, Status>> + Send>>;
    async fn watch_pose(
        &self,
        request: Request<WatchPoseRequest>,
    ) -> Result<Response<Self::WatchPoseStream>, Status> {
        let requested = request.into_inner();
        let initial = self.latest.borrow();
        if requested.reference_frame != initial.message.reference_frame
            || requested.tracked_frame != initial.message.tracked_frame
        {
            return Err(Status::invalid_argument(
                "requested pose frames do not match the configured producer",
            ));
        }
        let mut latest = self.latest.clone();
        let mut stop = self.stop.clone();
        let stream = async_stream::try_stream! {
            loop {
                if *stop.borrow() { break; }
                let sample = latest.borrow_and_update().clone();
                let mut message = sample.message;
                message.sample_age_ms = sample.captured.elapsed().as_millis() as u64;
                yield message;
                tokio::select! {
                    result = latest.changed() => if result.is_err() { break; },
                    _ = stop.changed() => break,
                }
            }
        };
        Ok(Response::new(Box::pin(stream)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_stream::StreamExt;

    #[tokio::test]
    async fn slow_consumers_get_latest_pose_and_shutdown_ends_stream() {
        let initial = Sample {
            message: WatchPoseResponse {
                reference_frame: "world".into(),
                tracked_frame: "base".into(),
                ..Default::default()
            },
            captured: Instant::now(),
        };
        let (tx, rx) = watch::channel(initial.clone());
        let (stop, stopped) = watch::channel(false);
        let service = Service {
            latest: rx,
            stop: stopped,
        };
        let mut stream = service
            .watch_pose(Request::new(WatchPoseRequest {
                reference_frame: "world".into(),
                tracked_frame: "base".into(),
            }))
            .await
            .unwrap()
            .into_inner();
        assert_eq!(stream.next().await.unwrap().unwrap().sequence, 0);
        for sequence in 1..1000 {
            let mut sample = initial.clone();
            sample.message.sequence = sequence;
            tx.send_replace(sample);
        }
        assert_eq!(stream.next().await.unwrap().unwrap().sequence, 999);
        stop.send_replace(true);
        assert!(stream.next().await.is_none());
        assert!(
            service
                .watch_pose(Request::new(WatchPoseRequest {
                    reference_frame: "wrong".into(),
                    tracked_frame: "base".into()
                }))
                .await
                .is_err()
        );
    }
}
