use std::collections::BTreeSet;
use std::path::Path;
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use bello_hardware_client::{Robot, RobotStateChange};
use bello_pose_api::pose_service_server::{PoseService, PoseServiceServer};
use bello_pose_api::{Orientation, Translation, WatchPoseRequest, WatchPoseResponse};
use futures_core::Stream;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tokio::net::TcpListener;
use tokio::sync::{oneshot, watch};
use tokio_stream::wrappers::TcpListenerStream;
use tonic::{Request, Response, Status};
use tracing::{debug, info, warn};

use crate::alignment::{CoordinateHistory, source_time_ns};
use crate::config::EstimatorConfig;
use crate::filter::{ContactFilter, PoseEstimate};
use crate::imu_filter::ImuLowPass;
use crate::model::KinematicModel;

#[derive(Clone, Debug)]
struct StampedPose {
    sequence: u64,
    pose: PoseEstimate,
    produced: Instant,
    source_age: Duration,
}

#[derive(Clone)]
struct PoseServer {
    reference_frame: Arc<str>,
    tracked_frame: Arc<str>,
    poses: watch::Receiver<Option<StampedPose>>,
}

#[tonic::async_trait]
impl PoseService for PoseServer {
    type WatchPoseStream =
        Pin<Box<dyn Stream<Item = Result<WatchPoseResponse, Status>> + Send + 'static>>;

    async fn watch_pose(
        &self,
        request: Request<WatchPoseRequest>,
    ) -> Result<Response<Self::WatchPoseStream>, Status> {
        let request = request.into_inner();
        if request.reference_frame != self.reference_frame.as_ref()
            || request.tracked_frame != self.tracked_frame.as_ref()
        {
            return Err(Status::invalid_argument(format!(
                "configured pose is {:?}_from_{:?}, requested {:?}_from_{:?}",
                self.reference_frame,
                self.tracked_frame,
                request.reference_frame,
                request.tracked_frame
            )));
        }
        let reference_frame = self.reference_frame.clone();
        let tracked_frame = self.tracked_frame.clone();
        let mut poses = self.poses.clone();
        let stream = async_stream::try_stream! {
            let mut last_sequence = None;
            loop {
                let current = poses.borrow_and_update().clone();
                if let Some(sample) = current
                    && last_sequence != Some(sample.sequence)
                {
                    last_sequence = Some(sample.sequence);
                    yield response_for(&reference_frame, &tracked_frame, sample);
                }
                if poses.changed().await.is_err() {
                    break;
                }
            }
        };
        Ok(Response::new(Box::pin(stream)))
    }
}

pub async fn run(path: &Path) -> anyhow::Result<()> {
    let (config, model) = load(path)?;
    let listen = config.listen;
    let listener = TcpListener::bind(listen)
        .await
        .with_context(|| format!("bind pose service to {listen}"))?;
    let robot = Robot::connect(config.hardware_endpoint.clone())
        .await
        .with_context(|| format!("connect to hardware at {}", config.hardware_endpoint))?;
    validate_inventory(&robot, &config, &model)?;

    let (pose_sender, pose_receiver) = watch::channel(None);
    let pose_server = PoseServer {
        reference_frame: Arc::from(config.reference_frame.as_str()),
        tracked_frame: Arc::from(config.tracked_frame.as_str()),
        poses: pose_receiver,
    };
    let (server_shutdown, server_shutdown_receiver) = oneshot::channel();
    let mut server = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(PoseServiceServer::new(pose_server))
            .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async move {
                let _ = server_shutdown_receiver.await;
            })
            .await
    });
    info!(%listen, "Bello pose estimator ready");

    let (monitor_shutdown, monitor_shutdown_receiver) = watch::channel(false);
    let mut monitor = tokio::spawn(monitor_hardware(
        robot,
        config,
        model,
        pose_sender,
        monitor_shutdown_receiver,
    ));
    let result = tokio::select! {
        result = &mut monitor => {
            match result {
                Ok(result) => result,
                Err(error) => Err(anyhow::anyhow!("hardware monitor task failed: {error}")),
            }
        }
        result = &mut server => {
            match result {
                Ok(Ok(())) => Err(anyhow::anyhow!("pose server stopped unexpectedly")),
                Ok(Err(error)) => Err(anyhow::anyhow!("pose server failed: {error}")),
                Err(error) => Err(anyhow::anyhow!("pose server task failed: {error}")),
            }
        }
        signal = tokio::signal::ctrl_c() => {
            signal.context("install Ctrl-C handler")?;
            Ok(())
        }
    };
    let _ = monitor_shutdown.send(true);
    let _ = server_shutdown.send(());
    if !monitor.is_finished()
        && tokio::time::timeout(Duration::from_secs(1), &mut monitor)
            .await
            .is_err()
    {
        warn!("hardware monitor did not stop within the shutdown deadline; aborting it");
        monitor.abort();
        let _ = monitor.await;
    }
    let server_result = if server.is_finished() {
        Ok(())
    } else {
        match tokio::time::timeout(Duration::from_secs(1), &mut server).await {
            Ok(joined) => joined
                .context("pose server task failed")?
                .map_err(Into::into),
            Err(_) => {
                warn!("pose server did not stop within the shutdown deadline; aborting it");
                server.abort();
                let _ = server.await;
                Err(anyhow::anyhow!("pose server shutdown timed out"))
            }
        }
    };
    result.and(server_result)
}

pub fn check(path: &Path) -> anyhow::Result<()> {
    let _ = load(path)?;
    Ok(())
}

fn load(path: &Path) -> anyhow::Result<(EstimatorConfig, KinematicModel)> {
    let config = EstimatorConfig::load(path)?;
    let model = KinematicModel::load(
        &config.urdf,
        &config.tracked_frame,
        &config.left_foot,
        &config.right_foot,
    )?;
    Ok((config, model))
}

fn validate_inventory(
    robot: &Robot,
    config: &EstimatorConfig,
    model: &KinematicModel,
) -> anyhow::Result<()> {
    let coordinates = robot
        .motors()
        .map(|motor| motor.coordinate().to_owned())
        .collect::<BTreeSet<_>>();
    let missing = model
        .required_coordinates()
        .iter()
        .filter(|name| !coordinates.contains(name.as_str()))
        .cloned()
        .collect::<Vec<_>>();
    if !missing.is_empty() {
        bail!("hardware inventory is missing estimator coordinates {missing:?}");
    }
    if !robot.imus().any(|imu| imu.name() == config.imu) {
        bail!("hardware inventory has no IMU {:?}", config.imu);
    }
    Ok(())
}

async fn monitor_hardware(
    robot: Robot,
    config: EstimatorConfig,
    model: KinematicModel,
    poses: watch::Sender<Option<StampedPose>>,
    mut shutdown: watch::Receiver<bool>,
) -> anyhow::Result<()> {
    let mut states = robot.states().await.context("open hardware state stream")?;
    let mut filter = ContactFilter::new(&config);
    let mut imu_low_pass = ImuLowPass::new(
        config.orientation_lpf_cutoff_hz,
        config.gyro_lpf_cutoff_hz,
        config.acceleration_lpf_cutoff_hz,
    );
    let mut coordinate_history = CoordinateHistory::default();
    let mut generation = None;
    let mut previous_imu_ns = None;
    let mut previous_motor_epoch = None;
    let mut sequence = 0_u64;
    let mut last_diagnostics = Instant::now();

    loop {
        let state = tokio::select! {
            changed = states.changed() => match changed.context("read hardware state stream")? {
                Some(state) => state,
                None => bail!("hardware state stream ended"),
            },
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    return Ok(());
                }
                continue;
            }
        };
        if generation != Some(state.generation()) {
            generation = Some(state.generation());
            previous_imu_ns = None;
            previous_motor_epoch = None;
            imu_low_pass.reset();
            coordinate_history.clear();
            if state.generation() != 0 {
                warn!(
                    generation = state.generation(),
                    "hardware telemetry reconnected; preserving pose and discarding outage time"
                );
            }
        }
        let imu = match state.last_change() {
            Some(RobotStateChange::Motor(motor)) => {
                if model
                    .required_coordinates()
                    .iter()
                    .any(|name| name == &motor.coordinate)
                {
                    coordinate_history.observe(
                        &motor.coordinate,
                        motor.monotonic_ns,
                        motor.feedback_age,
                        motor.position,
                        motor.velocity,
                    );
                }
                continue;
            }
            Some(RobotStateChange::Imu(imu)) => imu,
            _ => continue,
        };
        if imu.imu != config.imu || imu.sample_age > config.imu_stale() {
            continue;
        }
        let orientation = normalized_orientation(imu.orientation)?;
        let angular_velocity = Vector3::new(
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        );
        let linear_acceleration = Vector3::new(
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        );
        if !angular_velocity
            .iter()
            .chain(linear_acceleration.iter())
            .all(|v| v.is_finite())
        {
            warn!("discarding non-finite IMU sample");
            continue;
        }

        let imu_source_ns = source_time_ns(imu.monotonic_ns, imu.sample_age);
        let filtered_imu = imu_low_pass.update(
            imu_source_ns,
            orientation,
            angular_velocity,
            linear_acceleration,
        );
        let orientation = filtered_imu.orientation;
        let angular_velocity = filtered_imu.angular_velocity;
        let linear_acceleration = filtered_imu.linear_acceleration;
        let Some(coordinates) = coordinate_history.sample_at(
            model.required_coordinates(),
            imu_source_ns,
            config.motor_stale(),
        ) else {
            continue;
        };
        let feet = match model.feet(&coordinates.values) {
            Ok(feet) => feet,
            Err(error) => {
                warn!(%error, "discarding invalid kinematic sample");
                continue;
            }
        };

        let mut pose = None;
        let new_motor_frame =
            previous_motor_epoch.is_none_or(|previous| coordinates.epoch_ns > previous);
        if !filter.initialized() {
            pose = Some(filter.initialize(orientation, angular_velocity, feet));
        } else if let Some(previous) = previous_imu_ns
            && imu_source_ns > previous
        {
            let interval = Duration::from_nanos(imu_source_ns - previous);
            if interval <= Duration::from_millis(100) {
                pose = Some(filter.propagate(orientation, linear_acceleration, interval)?);
            }
        }
        if new_motor_frame {
            let interval = previous_motor_epoch
                .and_then(|previous| coordinates.epoch_ns.checked_sub(previous))
                .map(Duration::from_nanos)
                .filter(|interval| *interval <= Duration::from_millis(500))
                .unwrap_or(Duration::from_millis(10));
            let correction =
                filter.correct_contacts(orientation, angular_velocity, feet, interval)?;
            log_contact_diagnostics(&correction.contacts);
            pose = Some(correction.pose);
            previous_motor_epoch = Some(coordinates.epoch_ns);
        }
        previous_imu_ns = Some(imu_source_ns);
        if last_diagnostics.elapsed() >= Duration::from_secs(1) {
            let diagnostics = filter.diagnostics();
            debug!(
                position = ?diagnostics.position,
                velocity = ?diagnostics.velocity,
                accelerometer_bias = ?diagnostics.accelerometer_bias,
                contact_positions = ?diagnostics.contact_positions,
                active_contacts = ?diagnostics.active_contacts,
                contact_probabilities = ?diagnostics.contact_probabilities,
                effective_contact_probabilities = ?diagnostics.effective_contact_probabilities,
                motor_alignment_ms = coordinates.maximum_offset.as_secs_f64() * 1000.0,
                "pose estimator diagnostics"
            );
            last_diagnostics = Instant::now();
        }
        if let Some(pose) = pose {
            sequence = sequence.saturating_add(1);
            poses.send_replace(Some(StampedPose {
                sequence,
                pose,
                produced: Instant::now(),
                source_age: imu.sample_age,
            }));
        }
    }
}

fn log_contact_diagnostics(contacts: &[crate::filter::ContactDiagnostics; 2]) {
    for (foot, contact) in ["left", "right"].into_iter().zip(contacts) {
        if contact.changed {
            info!(
                foot,
                active = contact.active,
                probability = contact.probability,
                effective_probability = contact.effective_probability,
                height = contact.height,
                relative_height = contact.relative_height,
                world_speed = contact.world_speed,
                kinematic_speed = contact.kinematic_speed,
                contact_motion_speed = contact.contact_motion_speed,
                world_speed_reliability = contact.world_speed_reliability,
                "pose-estimator contact changed"
            );
        }
    }
}

fn normalized_orientation(
    orientation: bello_hardware_client::Quaternion,
) -> anyhow::Result<UnitQuaternion<f64>> {
    let quaternion = Quaternion::new(orientation.w, orientation.x, orientation.y, orientation.z);
    let norm = quaternion.norm();
    if !norm.is_finite() || norm <= 1.0e-12 {
        bail!("IMU quaternion is non-finite or has near-zero norm");
    }
    Ok(UnitQuaternion::new_normalize(quaternion))
}

fn response_for(
    reference_frame: &str,
    tracked_frame: &str,
    sample: StampedPose,
) -> WatchPoseResponse {
    let quaternion = sample.pose.orientation.quaternion();
    let age = sample.source_age.saturating_add(sample.produced.elapsed());
    WatchPoseResponse {
        reference_frame: reference_frame.to_owned(),
        tracked_frame: tracked_frame.to_owned(),
        sequence: sample.sequence,
        translation_metres: Some(Translation {
            x: sample.pose.position.x,
            y: sample.pose.position.y,
            z: sample.pose.position.z,
        }),
        orientation_xyzw: Some(Orientation {
            x: quaternion.i,
            y: quaternion.j,
            z: quaternion.k,
            w: quaternion.w,
        }),
        sample_age_ms: age.as_millis().min(u64::MAX as u128) as u64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_stream::StreamExt;

    #[tokio::test]
    async fn watch_pose_rejects_wrong_frames_and_streams_latest_pose() {
        let (sender, receiver) = watch::channel(None);
        let server = PoseServer {
            reference_frame: Arc::from("odom"),
            tracked_frame: Arc::from("pelvis"),
            poses: receiver,
        };
        let wrong = server
            .watch_pose(Request::new(WatchPoseRequest {
                reference_frame: "map".into(),
                tracked_frame: "pelvis".into(),
            }))
            .await
            .err()
            .expect("wrong frames must be rejected");
        assert_eq!(wrong.code(), tonic::Code::InvalidArgument);

        sender.send_replace(Some(StampedPose {
            sequence: 7,
            pose: PoseEstimate {
                position: Vector3::new(1.0, 2.0, 3.0),
                orientation: UnitQuaternion::identity(),
            },
            produced: Instant::now(),
            source_age: Duration::from_millis(4),
        }));
        let response = server
            .watch_pose(Request::new(WatchPoseRequest {
                reference_frame: "odom".into(),
                tracked_frame: "pelvis".into(),
            }))
            .await
            .unwrap();
        let sample = response.into_inner().next().await.unwrap().unwrap();
        assert_eq!(sample.sequence, 7);
        assert_eq!(sample.translation_metres.unwrap().z, 3.0);
        assert!(sample.sample_age_ms >= 4);
    }
}
