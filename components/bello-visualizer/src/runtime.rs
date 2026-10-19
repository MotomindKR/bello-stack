use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use bello_hardware_client::{
    CommandBehavior, ConnectOptions, ControlOptions, CoordinateTarget, DeviceState, DeviceStatus,
    GroupCause, GroupState, Quaternion, Robot, RobotStateChange, StateStreamOptions, TlsOptions,
    Vector3,
};
use bello_pose_api::WatchPoseRequest;
use bello_pose_api::pose_service_client::PoseServiceClient;
use parking_lot::RwLock;
use tokio::sync::watch;
use tonic::transport::{Certificate, Channel, ClientTlsConfig, Endpoint, Identity};

use crate::RobotDescription;
use crate::Transform;
use crate::config::{
    CoordinateLimits, EndpointConfig, GroupControlProfile, ResolvedGroupControlProfile, RootConfig,
    TlsConfig, VisualizerConfig,
};

const DIAGNOSTIC_EVENT_CAPACITY: usize = 512;
const DEFAULT_DISPLAY_STALE_AFTER: Duration = Duration::from_millis(200);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DisplayDeviceStatus {
    pub state: DeviceState,
    pub detail: String,
    pub transition_count: u64,
}

#[derive(Clone, Debug)]
pub struct DisplayBus {
    pub name: String,
    pub transport: String,
    pub host_id: u32,
    pub status: Option<DisplayDeviceStatus>,
}

#[derive(Clone, Debug)]
pub struct DisplaySerialDevice {
    pub name: String,
    pub port: String,
    pub baud_rate: u32,
    pub status: Option<DisplayDeviceStatus>,
}

#[derive(Clone, Debug)]
pub struct DisplayMotor {
    pub motor: String,
    pub coordinate: String,
    pub group: String,
    pub bus: String,
    pub id: u32,
    pub model: String,
    pub direction: i32,
    pub limits: CoordinateLimits,
    pub status: Option<DisplayDeviceStatus>,
    pub feedback_available: bool,
    pub feedback_current_connection: bool,
    pub monotonic_ns: Option<u64>,
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub temperature: f64,
    pub fault_bits: u32,
    pub mode_status: u32,
    pub age: Duration,
    pub stale_after: Duration,
}

#[derive(Clone, Debug)]
pub struct DisplayGroup {
    pub group: String,
    pub control_rate_hz: f64,
    pub state: GroupState,
    pub telemetry_available: bool,
    pub telemetry_current_connection: bool,
    pub monotonic_ns: Option<u64>,
    pub age: Option<Duration>,
    pub detail: String,
    pub cause: GroupCause,
    pub command_count: u64,
    pub control_tick: u64,
    pub jitter_us: i64,
}

#[derive(Clone, Debug)]
pub struct DisplayImu {
    pub imu: String,
    pub serial_device: String,
    pub status: Option<DisplayDeviceStatus>,
    pub sample_available: bool,
    pub sample_current_connection: bool,
    pub monotonic_ns: Option<u64>,
    pub orientation: Quaternion,
    pub angular_velocity: Vector3,
    pub linear_acceleration: Vector3,
    pub device_timestamp: Duration,
    pub sample_age: Duration,
}

#[derive(Clone, Debug)]
pub struct DisplayHealth {
    pub component: String,
    pub status: Option<DisplayDeviceStatus>,
    pub dropped_events: u64,
    pub monotonic_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagnosticSeverity {
    Info,
    Warning,
    Error,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiagnosticEvent {
    pub id: u64,
    pub monotonic_ns: Option<u64>,
    pub severity: DiagnosticSeverity,
    pub category: &'static str,
    pub component: String,
    pub detail: String,
    pub repeat_count: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ControlStatus {
    Monitoring,
    Starting,
    Running,
    Recovering,
    Stopping,
    Failed(String),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateCommand {
    pub position: f64,
    pub velocity: f64,
    pub torque_feedforward: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Clone, Debug)]
pub struct DisplaySnapshot {
    pub hardware_connected: bool,
    pub hardware_warning: Option<String>,
    pub buses: BTreeMap<String, DisplayBus>,
    pub serial_devices: BTreeMap<String, DisplaySerialDevice>,
    pub motors: BTreeMap<String, DisplayMotor>,
    pub groups: BTreeMap<String, DisplayGroup>,
    pub imus: BTreeMap<String, DisplayImu>,
    pub power_boards: BTreeMap<String, bello_hardware_client::PowerBoardSnapshot>,
    pub health: BTreeMap<String, DisplayHealth>,
    pub events: Vec<DiagnosticEvent>,
    pub telemetry_dropped_total: u64,
    pub events_evicted: u64,
    pub control: BTreeMap<String, ControlStatus>,
    pub root_transform: Transform,
    pub pose_available: bool,
    pub pose_warning: Option<String>,
    pub generation: u64,
}

impl Default for DisplaySnapshot {
    fn default() -> Self {
        Self {
            hardware_connected: false,
            hardware_warning: Some("waiting for hardware telemetry".into()),
            buses: BTreeMap::new(),
            serial_devices: BTreeMap::new(),
            motors: BTreeMap::new(),
            groups: BTreeMap::new(),
            imus: BTreeMap::new(),
            power_boards: BTreeMap::new(),
            health: BTreeMap::new(),
            events: Vec::new(),
            telemetry_dropped_total: 0,
            events_evicted: 0,
            control: BTreeMap::new(),
            root_transform: Transform::IDENTITY,
            pose_available: true,
            pose_warning: None,
            generation: 0,
        }
    }
}

struct RawMotor {
    value: DisplayMotor,
    received: Option<Instant>,
}

struct RawImu {
    value: DisplayImu,
    received: Option<Instant>,
}

#[derive(Default)]
struct RawHardware {
    connected: bool,
    warning: Option<String>,
    buses: BTreeMap<String, DisplayBus>,
    serial_devices: BTreeMap<String, DisplaySerialDevice>,
    motors: BTreeMap<String, RawMotor>,
    groups: BTreeMap<String, DisplayGroup>,
    group_received: BTreeMap<String, Instant>,
    imus: BTreeMap<String, RawImu>,
    power_boards: BTreeMap<String, bello_hardware_client::PowerBoardSnapshot>,
    health: BTreeMap<String, DisplayHealth>,
    events: VecDeque<DiagnosticEvent>,
    telemetry_dropped_total: u64,
    events_evicted: u64,
    next_event_id: u64,
}

pub struct RuntimeHandle {
    latest: Arc<RwLock<DisplaySnapshot>>,
    shutdown: watch::Sender<bool>,
    control: crossbeam_channel::Sender<ControlCommand>,
    thread: Option<thread::JoinHandle<()>>,
}

impl RuntimeHandle {
    pub fn spawn(config: VisualizerConfig, description: RobotDescription) -> anyhow::Result<Self> {
        let latest = Arc::new(RwLock::new(DisplaySnapshot::default()));
        if let RootConfig::Fixed { transform } = &config.root {
            let snapshot = &mut *latest.write();
            snapshot.root_transform = transform.checked("fixed root")?;
            snapshot.pose_available = true;
        }
        let (shutdown, shutdown_receiver) = watch::channel(false);
        let (control, control_receiver) = crossbeam_channel::unbounded();
        let thread_latest = Arc::clone(&latest);
        let thread = thread::Builder::new()
            .name("bello-visualizer-io".into())
            .spawn(move || {
                let runtime = tokio::runtime::Builder::new_multi_thread()
                    .enable_all()
                    .build()
                    .expect("visualizer Tokio runtime");
                runtime.block_on(async move {
                    let display_staleness = config
                        .control_profiles
                        .iter()
                        .map(|(group, profile)| {
                            (
                                group.clone(),
                                Duration::from_millis(profile.telemetry_staleness_ms),
                            )
                        })
                        .collect();
                    let hardware = tokio::spawn(monitor_hardware(
                        config.hardware.clone(),
                        config.refresh_hz,
                        display_staleness,
                        Arc::clone(&thread_latest),
                        shutdown_receiver.clone(),
                    ));
                    let pose = match config.root {
                        RootConfig::Fixed { .. } => None,
                        external @ RootConfig::ExternalPose { .. } => {
                            Some(tokio::spawn(monitor_pose(
                                external,
                                Arc::clone(&thread_latest),
                                shutdown_receiver.clone(),
                            )))
                        }
                    };
                    let controls = tokio::spawn(control_supervisor(
                        config.hardware,
                        config.control_profiles,
                        description,
                        control_receiver,
                        Arc::clone(&thread_latest),
                        shutdown_receiver.clone(),
                    ));
                    let _ = hardware.await;
                    if let Some(pose) = pose {
                        let _ = pose.await;
                    }
                    let _ = controls.await;
                });
            })?;
        Ok(Self {
            latest,
            shutdown,
            control,
            thread: Some(thread),
        })
    }

    pub fn latest(&self) -> DisplaySnapshot {
        self.latest.read().clone()
    }

    /// Starts selected groups. Calling this is an explicit lease-acquisition action.
    pub fn start_groups(&self, groups: Vec<String>) -> Result<(), String> {
        self.control
            .send(ControlCommand::Start(groups))
            .map_err(|_| "control runtime is closed".into())
    }

    pub fn set_targets(
        &self,
        group: impl Into<String>,
        targets: BTreeMap<String, CoordinateCommand>,
    ) -> Result<(), String> {
        self.control
            .send(ControlCommand::Targets(group.into(), targets))
            .map_err(|_| "control runtime is closed".into())
    }

    /// Acquires a temporary lease, explicitly recovers each selected group, then releases it.
    pub fn recover_groups(&self, groups: Vec<String>) -> Result<(), String> {
        self.control
            .send(ControlCommand::Recover(groups))
            .map_err(|_| "control runtime is closed".into())
    }

    pub fn stop_all(&self) -> Result<(), String> {
        self.control
            .send(ControlCommand::StopAll)
            .map_err(|_| "control runtime is closed".into())
    }

    /// Requests bounded background shutdown. A timed-out thread is detached rather than blocking UI exit.
    pub fn shutdown(mut self, timeout: Duration) -> bool {
        let _ = self.shutdown.send(true);
        let Some(thread) = self.thread.take() else {
            return true;
        };
        let (done_sender, done_receiver) = std::sync::mpsc::sync_channel(1);
        thread::spawn(move || {
            let _ = thread.join();
            let _ = done_sender.send(());
        });
        done_receiver.recv_timeout(timeout).is_ok()
    }
}

enum ControlCommand {
    Start(Vec<String>),
    Recover(Vec<String>),
    Targets(String, BTreeMap<String, CoordinateCommand>),
    StopAll,
}

struct ControlWorker {
    targets: watch::Sender<Option<BTreeMap<String, CoordinateCommand>>>,
    profile: ResolvedGroupControlProfile,
}

impl Drop for RuntimeHandle {
    fn drop(&mut self) {
        let _ = self.shutdown.send(true);
    }
}

async fn control_supervisor(
    endpoint: EndpointConfig,
    profiles: BTreeMap<String, GroupControlProfile>,
    description: RobotDescription,
    commands: crossbeam_channel::Receiver<ControlCommand>,
    latest: Arc<RwLock<DisplaySnapshot>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let (finished_sender, mut finished) = tokio::sync::mpsc::unbounded_channel::<String>();
    let mut workers = BTreeMap::<String, ControlWorker>::new();
    let mut robot = None;
    let mut poll = tokio::time::interval(Duration::from_millis(10));
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = poll.tick() => {
                while let Ok(command) = commands.try_recv() {
                    match command {
                        ControlCommand::Start(groups) => {
                            if robot.is_none() {
                                match connect_hardware(&endpoint).await {
                                    Ok(connected) => robot = Some(connected),
                                    Err(error) => {
                                        let mut display = latest.write();
                                        for group in groups {
                                            display.control.insert(group, ControlStatus::Failed(format!("connect for control: {error}")));
                                        }
                                        continue;
                                    }
                                }
                            }
                            for group_name in groups {
                                if workers.contains_key(&group_name) {
                                    latest.write().control.insert(
                                        group_name,
                                        ControlStatus::Failed(
                                            "group already has an active control or recovery session"
                                                .into(),
                                        ),
                                    );
                                    continue;
                                }
                                let Some(profile_config) = profiles.get(&group_name) else {
                                    latest.write().control.insert(
                                        group_name,
                                        ControlStatus::Failed("no explicit control profile".into()),
                                    );
                                    continue;
                                };
                                let connected = robot.as_ref().expect("connected above").clone();
                                let group = match connected.group(&group_name) {
                                    Ok(group) => group,
                                    Err(error) => {
                                        latest.write().control.insert(group_name, ControlStatus::Failed(error.to_string()));
                                        continue;
                                    }
                                };
                                let profile = match profile_config.resolve(&description, group.motors()) {
                                    Ok(profile) => profile,
                                    Err(error) => {
                                        latest.write().control.insert(group_name, ControlStatus::Failed(error.to_string()));
                                        continue;
                                    }
                                };
                                let initial = match seed_targets(&group_name, &profile, &latest) {
                                    Ok(initial) => initial,
                                    Err(error) => {
                                        latest.write().control.insert(group_name, ControlStatus::Failed(error));
                                        continue;
                                    }
                                };
                                let (sender, receiver) = watch::channel(Some(initial.clone()));
                                workers.insert(group_name.clone(), ControlWorker {
                                    targets: sender,
                                    profile: profile.clone(),
                                });
                                latest.write().control.insert(group_name.clone(), ControlStatus::Starting);
                                tokio::spawn(run_group_control(
                                    group,
                                    profile,
                                    initial,
                                    receiver,
                                    Arc::clone(&latest),
                                    shutdown.clone(),
                                    finished_sender.clone(),
                                ));
                            }
                        }
                        ControlCommand::Recover(groups) => {
                            if robot.is_none() {
                                match connect_hardware(&endpoint).await {
                                    Ok(connected) => robot = Some(connected),
                                    Err(error) => {
                                        let mut display = latest.write();
                                        for group in groups {
                                            display.control.insert(group, ControlStatus::Failed(format!("connect for recovery: {error}")));
                                        }
                                        continue;
                                    }
                                }
                            }
                            for group_name in groups {
                                if workers.contains_key(&group_name) {
                                    latest.write().control.insert(
                                        group_name,
                                        ControlStatus::Failed("group already has an active control or recovery session".into()),
                                    );
                                    continue;
                                }
                                let connected = robot.as_ref().expect("connected above").clone();
                                let group = match connected.group(&group_name) {
                                    Ok(group) => group,
                                    Err(error) => {
                                        latest.write().control.insert(group_name, ControlStatus::Failed(error.to_string()));
                                        continue;
                                    }
                                };
                                let (sender, _receiver) = watch::channel(None);
                                let profile = profiles
                                    .get(&group_name)
                                    .and_then(|profile| profile.resolve(&description, group.motors()).ok());
                                let Some(profile) = profile else {
                                    latest.write().control.insert(group_name, ControlStatus::Failed("could not resolve live hardware coordinate limits for recovery group".into()));
                                    continue;
                                };
                                workers.insert(group_name.clone(), ControlWorker { targets: sender, profile });
                                latest.write().control.insert(group_name.clone(), ControlStatus::Recovering);
                                tokio::spawn(run_group_recovery(
                                    group,
                                    Arc::clone(&latest),
                                    finished_sender.clone(),
                                ));
                            }
                        }
                        ControlCommand::Targets(group, targets) => {
                            if let Some(worker) = workers.get(&group) {
                                if let Err(error) = validate_targets(&group, &worker.profile, &targets) {
                                    let _ = worker.targets.send(None);
                                    latest.write().control.insert(group, ControlStatus::Failed(error));
                                } else {
                                    let _ = worker.targets.send(Some(targets));
                                }
                            }
                        }
                        ControlCommand::StopAll => stop_workers(&workers, &latest),
                    }
                }
            }
            Some(group) = finished.recv() => {
                workers.remove(&group);
            }
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    stop_workers(&workers, &latest);
                    while !workers.is_empty() {
                        match tokio::time::timeout(Duration::from_secs(2), finished.recv()).await {
                            Ok(Some(group)) => { workers.remove(&group); }
                            _ => break,
                        }
                    }
                    return;
                }
            }
        }
    }
}

async fn run_group_recovery(
    group: bello_hardware_client::MotorGroup,
    latest: Arc<RwLock<DisplaySnapshot>>,
    finished: tokio::sync::mpsc::UnboundedSender<String>,
) {
    let group_name = group.name().to_owned();
    let result = async {
        let mut session = group.control("bello-visualizer-recovery").await?;
        let recovery = session.recover().await;
        let cleanup = session.shutdown().await;
        recovery?;
        cleanup?;
        anyhow::Ok(())
    }
    .await;
    latest.write().control.insert(
        group_name.clone(),
        match result {
            Ok(()) => ControlStatus::Monitoring,
            Err(error) => ControlStatus::Failed(error.to_string()),
        },
    );
    let _ = finished.send(group_name);
}

fn stop_workers(workers: &BTreeMap<String, ControlWorker>, latest: &RwLock<DisplaySnapshot>) {
    let mut display = latest.write();
    for (group, worker) in workers {
        let _ = worker.targets.send(None);
        display
            .control
            .insert(group.clone(), ControlStatus::Stopping);
    }
}

fn seed_targets(
    group: &str,
    profile: &ResolvedGroupControlProfile,
    latest: &RwLock<DisplaySnapshot>,
) -> Result<BTreeMap<String, CoordinateCommand>, String> {
    let display = latest.read();
    let maximum_age = Duration::from_millis(profile.telemetry_staleness_ms);
    profile
        .coordinates
        .iter()
        .map(|(coordinate, limits)| {
            let feedback = display
                .motors
                .get(coordinate)
                .filter(|feedback| {
                    feedback.group == group
                        && feedback.feedback_available
                        && feedback.feedback_current_connection
                })
                .ok_or_else(|| format!("no feedback for coordinate {coordinate:?}"))?;
            if feedback.age > maximum_age {
                return Err(format!(
                    "feedback for coordinate {coordinate:?} is stale ({:?})",
                    feedback.age
                ));
            }
            let Some(position) = limits.position.clamp_feedback_position(feedback.position) else {
                return Err(format!(
                    "feedback position for coordinate {coordinate:?} is not finite ({})",
                    feedback.position
                ));
            };
            Ok((
                coordinate.clone(),
                CoordinateCommand {
                    position,
                    velocity: limits.velocity.default,
                    torque_feedforward: limits.torque_feedforward.default,
                    kp: limits.kp.default,
                    kd: limits.kd.default,
                },
            ))
        })
        .collect()
}

fn validate_targets(
    group: &str,
    profile: &ResolvedGroupControlProfile,
    targets: &BTreeMap<String, CoordinateCommand>,
) -> Result<(), String> {
    if targets.len() != profile.coordinates.len() {
        return Err(format!("target frame for {group:?} is incomplete"));
    }
    for (coordinate, limits) in &profile.coordinates {
        let target = targets
            .get(coordinate)
            .ok_or_else(|| format!("target frame is missing coordinate {coordinate:?}"))?;
        for (field, value, bounds) in [
            ("position", target.position, limits.position),
            ("velocity", target.velocity, limits.velocity),
            (
                "torque_feedforward",
                target.torque_feedforward,
                limits.torque_feedforward,
            ),
            ("kp", target.kp, limits.kp),
            ("kd", target.kd, limits.kd),
        ] {
            if !value.is_finite() || value < bounds.min || value > bounds.max {
                return Err(format!(
                    "{field} target {value} for {coordinate:?} is outside [{}, {}]",
                    bounds.min, bounds.max
                ));
            }
        }
    }
    Ok(())
}

async fn run_group_control(
    group: bello_hardware_client::MotorGroup,
    profile: ResolvedGroupControlProfile,
    initial: BTreeMap<String, CoordinateCommand>,
    mut targets: watch::Receiver<Option<BTreeMap<String, CoordinateCommand>>>,
    latest: Arc<RwLock<DisplaySnapshot>>,
    mut shutdown: watch::Receiver<bool>,
    finished: tokio::sync::mpsc::UnboundedSender<String>,
) {
    let group_name = group.name().to_owned();
    let options = ControlOptions::new("bello-visualizer")
        .command_validity(Duration::from_millis(profile.validity_ms));
    let result = async {
        let initial_frame = build_frame(&group, &initial)?;
        let mut commanded = initial.clone();
        let mut session = group.control_with(options).await?;
        if targets.borrow().is_none() {
            session.shutdown().await?;
            return anyhow::Ok(());
        }
        if let Err(error) = session.start(initial_frame).await {
            let cleanup = session.shutdown().await;
            return Err(match cleanup {
                Ok(()) => error.into(),
                Err(cleanup) => anyhow::anyhow!("{error}; cleanup also failed: {cleanup}"),
            });
        }
        latest
            .write()
            .control
            .insert(group_name.clone(), ControlStatus::Running);
        let Some(mut current) = targets.borrow().clone() else {
            session.shutdown().await?;
            return anyhow::Ok(());
        };
        let mut interval =
            tokio::time::interval(Duration::from_secs_f64(1.0 / profile.command_rate_hz));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        interval.tick().await;
        let motion_result: anyhow::Result<()> = loop {
            tokio::select! {
                _ = interval.tick() => {
                    if let Err(error) = ensure_group_safe_and_fresh(&group_name, &profile, &latest) {
                        break Err(error);
                    }
                    slew_targets(
                        &mut commanded,
                        &current,
                        profile.position_slew_rate / profile.command_rate_hz,
                    );
                    let frame = match build_frame(&group, &commanded) {
                        Ok(frame) => frame,
                        Err(error) => break Err(error),
                    };
                    if let Err(error) = session.send(frame).await {
                        break Err(error.into());
                    }
                }
                changed = targets.changed() => {
                    if changed.is_err() {
                        break Ok(());
                    }
                    let next = targets.borrow().clone();
                    match next {
                        Some(next) => current = next,
                        None => break Ok(()),
                    }
                }
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break Ok(());
                    }
                }
            }
        };
        let cleanup = session.shutdown().await;
        match (motion_result, cleanup) {
            (Ok(()), Ok(())) => anyhow::Ok(()),
            (Err(error), Ok(())) => Err(error),
            (Ok(()), Err(error)) => Err(error.into()),
            (Err(error), Err(cleanup)) => {
                Err(anyhow::anyhow!("{error}; cleanup also failed: {cleanup}"))
            }
        }
    }
    .await;
    let mut display = latest.write();
    match result {
        Ok(())
            if !matches!(
                display.control.get(&group_name),
                Some(ControlStatus::Failed(_))
            ) =>
        {
            display
                .control
                .insert(group_name.clone(), ControlStatus::Monitoring);
        }
        Ok(()) => {}
        Err(error) => {
            display
                .control
                .insert(group_name.clone(), ControlStatus::Failed(error.to_string()));
        }
    }
    let _ = finished.send(group_name);
}

fn slew_targets(
    commanded: &mut BTreeMap<String, CoordinateCommand>,
    desired: &BTreeMap<String, CoordinateCommand>,
    maximum_position_step: f64,
) {
    for (coordinate, next) in desired {
        let Some(current) = commanded.get_mut(coordinate) else {
            continue;
        };
        let delta = next.position - current.position;
        let position =
            current.position + delta.clamp(-maximum_position_step, maximum_position_step);
        *current = *next;
        current.position = position;
    }
}

fn build_frame(
    group: &bello_hardware_client::MotorGroup,
    targets: &BTreeMap<String, CoordinateCommand>,
) -> anyhow::Result<bello_hardware_client::CommandFrame> {
    let mut frame = group.frame();
    for coordinate in group.coordinate_names() {
        let target = targets
            .get(coordinate)
            .ok_or_else(|| anyhow::anyhow!("missing coordinate {coordinate:?}"))?;
        frame = frame.coordinate(
            coordinate,
            CoordinateTarget::hybrid(
                target.position,
                target.velocity,
                target.torque_feedforward,
                target.kp,
                target.kd,
            ),
        )?;
    }
    Ok(frame.build()?)
}

fn ensure_group_safe_and_fresh(
    group: &str,
    profile: &ResolvedGroupControlProfile,
    latest: &RwLock<DisplaySnapshot>,
) -> anyhow::Result<()> {
    let display = latest.read();
    if display
        .groups
        .get(group)
        .is_some_and(|state| state.state == GroupState::Fault)
    {
        anyhow::bail!("group entered fault");
    }
    let maximum_age = Duration::from_millis(profile.telemetry_staleness_ms);
    for coordinate in profile.coordinates.keys() {
        let sample = display
            .motors
            .get(coordinate)
            .filter(|sample| {
                sample.group == group
                    && sample.feedback_available
                    && sample.feedback_current_connection
            })
            .ok_or_else(|| anyhow::anyhow!("feedback for {coordinate:?} is unavailable"))?;
        if sample.age > maximum_age {
            anyhow::bail!("feedback for {coordinate:?} is stale ({:?})", sample.age);
        }
        let limits = profile.coordinates[coordinate].position;
        if limits.clamp_feedback_position(sample.position).is_none() {
            anyhow::bail!(
                "feedback position for {coordinate:?} is not finite ({})",
                sample.position
            );
        }
    }
    Ok(())
}

async fn monitor_hardware(
    endpoint: EndpointConfig,
    refresh_hz: f64,
    display_staleness: BTreeMap<String, Duration>,
    latest: Arc<RwLock<DisplaySnapshot>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let raw = Arc::new(RwLock::new(RawHardware::default()));
    let publisher = tokio::spawn(publish_hardware(
        Arc::clone(&raw),
        Arc::clone(&latest),
        refresh_hz,
        shutdown.clone(),
    ));
    while !*shutdown.borrow() {
        match connect_hardware(&endpoint).await {
            Ok(robot) => {
                apply_robot_inventory(&mut raw.write(), &robot, &display_staleness);
                let options = StateStreamOptions::default().reconnect_on_unavailable(false);
                match robot.states_with(options).await {
                    Ok(mut states) => {
                        mark_hardware_connected(&raw);
                        loop {
                            tokio::select! {
                                changed = states.changed() => match changed {
                                    Ok(Some(state)) => apply_robot_state(&mut raw.write(), state),
                                    Ok(None) => {
                                        mark_hardware_disconnected(&raw, "hardware telemetry stream closed");
                                        break;
                                    }
                                    Err(error) => {
                                        mark_hardware_disconnected(&raw, format!("hardware telemetry: {error}"));
                                        break;
                                    }
                                },
                                changed = shutdown.changed() => {
                                    if changed.is_err() || *shutdown.borrow() {
                                        let _ = publisher.await;
                                        return;
                                    }
                                }
                            }
                        }
                    }
                    Err(error) => {
                        mark_hardware_disconnected(&raw, format!("open telemetry: {error}"));
                    }
                }
            }
            Err(error) => mark_hardware_disconnected(&raw, format!("connect hardware: {error}")),
        }
        if wait_or_shutdown(Duration::from_millis(500), &mut shutdown).await {
            break;
        }
    }
    let _ = publisher.await;
}

async fn publish_hardware(
    raw: Arc<RwLock<RawHardware>>,
    latest: Arc<RwLock<DisplaySnapshot>>,
    refresh_hz: f64,
    mut shutdown: watch::Receiver<bool>,
) {
    let period = Duration::from_secs_f64(1.0 / refresh_hz);
    let mut interval = tokio::time::interval(period);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = interval.tick() => {
                let now = Instant::now();
                let raw = raw.read();
                let mut display = latest.write();
                display.hardware_connected = raw.connected;
                display.hardware_warning = raw.warning.clone();
                display.buses = raw.buses.clone();
                display.serial_devices = raw.serial_devices.clone();
                display.groups = raw.groups.iter().map(|(name, group)| {
                    let mut value = group.clone();
                    value.age = raw.group_received.get(name).map(|received| now.saturating_duration_since(*received));
                    (name.clone(), value)
                }).collect();
                display.motors = raw.motors.iter().map(|(name, motor)| {
                    let mut value = motor.value.clone();
                    if let Some(received) = motor.received {
                        value.age = value.age.saturating_add(now.saturating_duration_since(received));
                    }
                    (name.clone(), value)
                }).collect();
                display.imus = raw.imus.iter().map(|(name, imu)| {
                    let mut value = imu.value.clone();
                    if let Some(received) = imu.received {
                        value.sample_age = value.sample_age.saturating_add(now.saturating_duration_since(received));
                    }
                    (name.clone(), value)
                }).collect();
                display.health = raw.health.clone();
                display.power_boards = raw.power_boards.clone();
                display.events = raw.events.iter().cloned().collect();
                display.telemetry_dropped_total = raw.telemetry_dropped_total;
                display.events_evicted = raw.events_evicted;
                display.generation = display.generation.wrapping_add(1);
            }
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    return;
                }
            }
        }
    }
}

async fn connect_hardware(config: &EndpointConfig) -> anyhow::Result<Robot> {
    let mut options = ConnectOptions::new(&config.endpoint);
    if let Some(tls) = &config.tls {
        let mut settings = TlsOptions::new();
        if let Some(path) = &tls.ca_certificate {
            settings = settings.ca_certificate(fs::read(path)?);
        }
        if let (Some(certificate), Some(key)) = (&tls.client_certificate, &tls.client_private_key) {
            settings = settings.identity(fs::read(certificate)?, fs::read(key)?);
        }
        options = options.tls(settings);
    }
    Ok(Robot::connect_with(options).await?)
}

fn mark_hardware_disconnected(raw: &RwLock<RawHardware>, warning: impl Into<String>) {
    let warning = warning.into();
    let raw = &mut *raw.write();
    if raw.connected || raw.warning.as_deref() != Some(&warning) {
        push_event(
            raw,
            DiagnosticEvent {
                id: 0,
                monotonic_ns: None,
                severity: DiagnosticSeverity::Error,
                category: "connection",
                component: "hardwared".into(),
                detail: warning.clone(),
                repeat_count: 1,
            },
        );
    }
    raw.connected = false;
    raw.power_boards.clear();
    raw.warning = Some(warning);
    for motor in raw.motors.values_mut() {
        motor.value.feedback_current_connection = false;
    }
    for group in raw.groups.values_mut() {
        group.telemetry_current_connection = false;
    }
    for imu in raw.imus.values_mut() {
        imu.value.sample_current_connection = false;
    }
}

fn mark_hardware_connected(raw: &RwLock<RawHardware>) {
    let raw = &mut *raw.write();
    if !raw.connected {
        push_event(
            raw,
            DiagnosticEvent {
                id: 0,
                monotonic_ns: None,
                severity: DiagnosticSeverity::Info,
                category: "connection",
                component: "hardwared".into(),
                detail: "hardware telemetry connected".into(),
                repeat_count: 1,
            },
        );
    }
    raw.connected = true;
    raw.warning = None;
}

fn apply_robot_inventory(
    raw: &mut RawHardware,
    robot: &Robot,
    display_staleness: &BTreeMap<String, Duration>,
) {
    // Inventory belongs to a new connection generation. Keep cumulative events
    // and drop totals, but current lifecycle rows must be rebuilt from this
    // connection's inventory/health stream.
    raw.health.clear();
    raw.buses = robot
        .buses()
        .map(|bus| {
            (
                bus.name().to_owned(),
                DisplayBus {
                    name: bus.name().to_owned(),
                    transport: bus.transport().to_owned(),
                    host_id: bus.host_id(),
                    status: bus.initial_status().map(display_device_status),
                },
            )
        })
        .collect();
    raw.serial_devices = robot
        .serial_devices()
        .map(|serial| {
            (
                serial.name().to_owned(),
                DisplaySerialDevice {
                    name: serial.name().to_owned(),
                    port: serial.port().to_owned(),
                    baud_rate: serial.baud_rate(),
                    status: serial.initial_status().map(display_device_status),
                },
            )
        })
        .collect();

    let mut previous_motors = std::mem::take(&mut raw.motors);
    raw.motors = robot
        .motors()
        .map(|motor| {
            let coordinate = motor.coordinate().to_owned();
            let previous = previous_motors.remove(&coordinate).filter(|previous| {
                let value = &previous.value;
                value.motor == motor.name()
                    && value.group == motor.group()
                    && value.bus == motor.bus()
                    && value.id == motor.id()
                    && value.model == motor.model()
                    && value.direction == motor.direction()
                    && value.limits == CoordinateLimits::from(motor)
            });
            let (
                feedback_available,
                monotonic_ns,
                position,
                velocity,
                effort,
                temperature,
                fault_bits,
                mode_status,
                age,
                received,
            ) = previous.map_or(
                (false, None, 0.0, 0.0, 0.0, 0.0, 0, 0, Duration::ZERO, None),
                |previous| {
                    let value = previous.value;
                    (
                        value.feedback_available,
                        value.monotonic_ns,
                        value.position,
                        value.velocity,
                        value.effort,
                        value.temperature,
                        value.fault_bits,
                        value.mode_status,
                        value.age,
                        previous.received,
                    )
                },
            );
            (
                coordinate.clone(),
                RawMotor {
                    value: DisplayMotor {
                        motor: motor.name().to_owned(),
                        coordinate,
                        group: motor.group().to_owned(),
                        bus: motor.bus().to_owned(),
                        id: motor.id(),
                        model: motor.model().to_owned(),
                        direction: motor.direction(),
                        limits: CoordinateLimits::from(motor),
                        status: motor.initial_status().map(display_device_status),
                        feedback_available,
                        feedback_current_connection: false,
                        monotonic_ns,
                        position,
                        velocity,
                        effort,
                        temperature,
                        fault_bits,
                        mode_status,
                        age,
                        stale_after: display_staleness
                            .get(motor.group())
                            .copied()
                            .unwrap_or(DEFAULT_DISPLAY_STALE_AFTER),
                    },
                    received,
                },
            )
        })
        .collect();

    raw.group_received.clear();
    raw.groups = robot
        .group_names()
        .filter_map(|name| robot.group(name).ok())
        .map(|group| {
            let name = group.name().to_owned();
            (
                name.clone(),
                DisplayGroup {
                    group: name,
                    control_rate_hz: group.control_rate_hz(),
                    state: group.initial_state(),
                    telemetry_available: false,
                    telemetry_current_connection: false,
                    monotonic_ns: None,
                    age: None,
                    detail: String::new(),
                    cause: GroupCause::Unspecified,
                    command_count: 0,
                    control_tick: 0,
                    jitter_us: 0,
                },
            )
        })
        .collect();

    let mut previous_imus = std::mem::take(&mut raw.imus);
    raw.imus = robot
        .imus()
        .map(|imu| {
            let name = imu.name().to_owned();
            let previous = previous_imus
                .remove(&name)
                .filter(|previous| previous.value.serial_device == imu.serial_device());
            let (
                sample_available,
                monotonic_ns,
                orientation,
                angular_velocity,
                linear_acceleration,
                device_timestamp,
                sample_age,
                received,
            ) = previous.map_or(
                (
                    false,
                    None,
                    Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                    zero_vector(),
                    zero_vector(),
                    Duration::ZERO,
                    Duration::ZERO,
                    None,
                ),
                |previous| {
                    let value = previous.value;
                    (
                        value.sample_available,
                        value.monotonic_ns,
                        value.orientation,
                        value.angular_velocity,
                        value.linear_acceleration,
                        value.device_timestamp,
                        value.sample_age,
                        previous.received,
                    )
                },
            );
            (
                name.clone(),
                RawImu {
                    value: DisplayImu {
                        imu: name,
                        serial_device: imu.serial_device().to_owned(),
                        status: imu.initial_status().map(display_device_status),
                        sample_available,
                        sample_current_connection: false,
                        monotonic_ns,
                        orientation,
                        angular_velocity,
                        linear_acceleration,
                        device_timestamp,
                        sample_age,
                    },
                    received,
                },
            )
        })
        .collect();
}

fn apply_robot_state(raw: &mut RawHardware, state: &bello_hardware_client::RobotState) {
    let Some(change) = state.last_change() else {
        return;
    };
    let now = Instant::now();
    match change {
        RobotStateChange::PowerBoard(board) => {
            raw.power_boards.insert(board.board.clone(), board.clone());
        }
        RobotStateChange::Motor(motor) => {
            let Some(target) = raw.motors.get_mut(&motor.coordinate) else {
                push_event(
                    raw,
                    protocol_event(
                        motor.monotonic_ns,
                        format!(
                            "telemetry references unknown coordinate {:?}",
                            motor.coordinate
                        ),
                    ),
                );
                return;
            };
            target.value.feedback_available = true;
            target.value.feedback_current_connection = true;
            target.value.monotonic_ns = Some(motor.monotonic_ns);
            target.value.position = motor.position;
            target.value.velocity = motor.velocity;
            target.value.effort = motor.torque;
            target.value.temperature = motor.temperature;
            target.value.fault_bits = motor.fault_bits;
            target.value.mode_status = motor.mode_status;
            target.value.age = motor.feedback_age;
            target.received = Some(now);
        }
        RobotStateChange::Imu(imu) => {
            let Some(target) = raw.imus.get_mut(&imu.imu) else {
                push_event(
                    raw,
                    protocol_event(
                        imu.monotonic_ns,
                        format!("telemetry references unknown IMU {:?}", imu.imu),
                    ),
                );
                return;
            };
            target.value.sample_available = true;
            target.value.sample_current_connection = true;
            target.value.monotonic_ns = Some(imu.monotonic_ns);
            target.value.orientation = imu.orientation;
            target.value.angular_velocity = imu.angular_velocity;
            target.value.linear_acceleration = imu.linear_acceleration;
            target.value.device_timestamp = imu.device_timestamp;
            target.value.sample_age = imu.sample_age;
            target.received = Some(now);
        }
        RobotStateChange::Group(group) => {
            let Some(target) = raw.groups.get_mut(&group.group) else {
                push_event(
                    raw,
                    protocol_event(
                        group.monotonic_ns,
                        format!("telemetry references unknown group {:?}", group.group),
                    ),
                );
                return;
            };
            let changed = !target.telemetry_available
                || target.state != group.state
                || target.cause != group.cause
                || target.detail != group.detail;
            target.state = group.state;
            target.telemetry_available = true;
            target.telemetry_current_connection = true;
            target.monotonic_ns = Some(group.monotonic_ns);
            target.detail = group.detail.clone();
            target.cause = group.cause;
            target.command_count = group.command_count;
            target.control_tick = group.control_tick;
            target.jitter_us = group.jitter_us;
            raw.group_received.insert(group.group.clone(), now);
            if changed {
                let detail = if group.detail.is_empty() {
                    format!("state changed to {:?}", group.state)
                } else {
                    format!("{:?}: {}", group.state, group.detail)
                };
                push_event(
                    raw,
                    DiagnosticEvent {
                        id: 0,
                        monotonic_ns: Some(group.monotonic_ns),
                        severity: if group.state == GroupState::Fault {
                            DiagnosticSeverity::Error
                        } else {
                            DiagnosticSeverity::Info
                        },
                        category: "group",
                        component: group.group.clone(),
                        detail,
                        repeat_count: 1,
                    },
                );
            }
        }
        RobotStateChange::Health(health) => {
            let status = health.status.as_ref().map(display_device_status);
            apply_component_status(raw, &health.component, status.clone());
            raw.health.insert(
                health.component.clone(),
                DisplayHealth {
                    component: health.component.clone(),
                    status: status.clone(),
                    dropped_events: health.dropped_events,
                    monotonic_ns: health.monotonic_ns,
                },
            );
            raw.telemetry_dropped_total = raw
                .telemetry_dropped_total
                .saturating_add(health.dropped_events);
            let detail = match &status {
                Some(status) if health.dropped_events > 0 => format!(
                    "{:?}: {}; {} telemetry events dropped",
                    status.state, status.detail, health.dropped_events
                ),
                Some(status) => format!("{:?}: {}", status.state, status.detail),
                None => format!("{} telemetry events dropped", health.dropped_events),
            };
            push_event(
                raw,
                DiagnosticEvent {
                    id: 0,
                    monotonic_ns: Some(health.monotonic_ns),
                    severity: status
                        .as_ref()
                        .map_or(DiagnosticSeverity::Warning, |status| {
                            device_severity(status.state)
                        }),
                    category: "health",
                    component: health.component.clone(),
                    detail,
                    repeat_count: 1,
                },
            );
        }
        RobotStateChange::CommandPolicy(policy) => {
            let severity =
                if policy.command_rejected || matches!(policy.behavior, CommandBehavior::Fault) {
                    DiagnosticSeverity::Error
                } else if policy.command_modified {
                    DiagnosticSeverity::Warning
                } else {
                    DiagnosticSeverity::Info
                };
            push_event(
                raw,
                DiagnosticEvent {
                    id: 0,
                    monotonic_ns: Some(policy.monotonic_ns),
                    severity,
                    category: "command policy",
                    component: format!("{}/{}", policy.group, policy.motor),
                    detail: format!(
                        "{:?} {:?}: {}; modified={} rejected={} response={:?}",
                        policy.check,
                        policy.behavior,
                        policy.detail,
                        policy.command_modified,
                        policy.command_rejected,
                        policy.fault_response
                    ),
                    repeat_count: 1,
                },
            );
        }
        _ => {}
    }
}

fn apply_component_status(
    raw: &mut RawHardware,
    component: &str,
    status: Option<DisplayDeviceStatus>,
) {
    let Some(status) = status else {
        return;
    };
    if let Some(bus) = raw
        .buses
        .values_mut()
        .find(|bus| component == format!("can/{}", bus.name))
    {
        bus.status = Some(status);
        return;
    }
    if let Some(serial) = raw
        .serial_devices
        .values_mut()
        .find(|serial| component == format!("serial/{}", serial.name))
    {
        serial.status = Some(status);
        return;
    }
    if let Some(motor) = raw
        .motors
        .values_mut()
        .find(|motor| component == format!("motor/{}", motor.value.motor))
    {
        motor.value.status = Some(status);
        return;
    }
    if let Some(imu) = raw
        .imus
        .values_mut()
        .find(|imu| component == format!("imu/{}", imu.value.imu))
    {
        imu.value.status = Some(status);
    }
}

fn display_device_status(status: &DeviceStatus) -> DisplayDeviceStatus {
    DisplayDeviceStatus {
        state: status.state(),
        detail: status.detail().to_owned(),
        transition_count: status.transition_count(),
    }
}

fn device_severity(state: DeviceState) -> DiagnosticSeverity {
    match state {
        DeviceState::Online => DiagnosticSeverity::Info,
        DeviceState::Fault => DiagnosticSeverity::Error,
        _ => DiagnosticSeverity::Warning,
    }
}

fn protocol_event(monotonic_ns: u64, detail: String) -> DiagnosticEvent {
    DiagnosticEvent {
        id: 0,
        monotonic_ns: Some(monotonic_ns),
        severity: DiagnosticSeverity::Error,
        category: "protocol",
        component: "hardwared".into(),
        detail,
        repeat_count: 1,
    }
}

fn push_event(raw: &mut RawHardware, mut event: DiagnosticEvent) {
    raw.next_event_id = raw.next_event_id.wrapping_add(1).max(1);
    event.id = raw.next_event_id;
    if let Some(index) = raw.events.iter().position(|existing| {
        existing.severity == event.severity
            && existing.category == event.category
            && existing.component == event.component
            && existing.detail == event.detail
    }) {
        let mut existing = raw
            .events
            .remove(index)
            .expect("event index came from the same deque");
        existing.id = event.id;
        existing.monotonic_ns = event.monotonic_ns.or(existing.monotonic_ns);
        existing.repeat_count = existing.repeat_count.saturating_add(event.repeat_count);
        raw.events.push_back(existing);
        return;
    }
    if raw.events.len() == DIAGNOSTIC_EVENT_CAPACITY {
        raw.events.pop_front();
        raw.events_evicted = raw.events_evicted.saturating_add(1);
    }
    raw.events.push_back(event);
}

const fn zero_vector() -> Vector3 {
    Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

async fn monitor_pose(
    config: RootConfig,
    latest: Arc<RwLock<DisplaySnapshot>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let RootConfig::ExternalPose {
        endpoint,
        tls,
        reference_frame,
        tracked_frame,
        tracked_from_model_root,
        staleness_ms,
    } = config
    else {
        return;
    };
    let maximum_age = Duration::from_millis(staleness_ms);
    while !*shutdown.borrow() {
        match connect_pose(&endpoint, tls.as_ref()).await {
            Ok(channel) => {
                let mut client = PoseServiceClient::new(channel);
                let request = WatchPoseRequest {
                    reference_frame: reference_frame.clone(),
                    tracked_frame: tracked_frame.clone(),
                };
                match client.watch_pose(request).await {
                    Ok(response) => {
                        let mut stream = response.into_inner();
                        let mut last_sequence = None;
                        let mut last_received = None::<(Instant, Duration)>;
                        let mut staleness_tick = tokio::time::interval(Duration::from_millis(50));
                        loop {
                            tokio::select! {
                                sample = stream.message() => match sample {
                                    Ok(Some(sample)) => {
                                        let result = validate_pose_sample(
                                            sample,
                                            &reference_frame,
                                            &tracked_frame,
                                            last_sequence,
                                        );
                                        match result {
                                            Ok((sequence, reference_from_tracked, producer_age)) => {
                                                last_sequence = Some(sequence);
                                                last_received = Some((Instant::now(), producer_age));
                                                let display = &mut *latest.write();
                                                if producer_age <= maximum_age {
                                                    display.root_transform = reference_from_tracked.compose(tracked_from_model_root);
                                                    display.pose_available = true;
                                                    display.pose_warning = None;
                                                } else {
                                                    display.pose_available = false;
                                                    display.pose_warning = Some(format!("pose sample is stale ({producer_age:?})"));
                                                }
                                            }
                                            Err(warning) => {
                                                let display = &mut *latest.write();
                                                display.pose_available = false;
                                                display.pose_warning = Some(warning);
                                            }
                                        }
                                    }
                                    Ok(None) => {
                                        mark_pose_unavailable(&latest, "pose stream closed");
                                        break;
                                    }
                                    Err(error) => {
                                        mark_pose_unavailable(&latest, format!("pose stream: {error}"));
                                        break;
                                    }
                                },
                                _ = staleness_tick.tick() => {
                                    if let Some((received, producer_age)) = last_received {
                                        let age = producer_age.saturating_add(received.elapsed());
                                        if age > maximum_age {
                                            mark_pose_unavailable(&latest, format!("pose sample is stale ({age:?})"));
                                        }
                                    }
                                }
                                changed = shutdown.changed() => {
                                    if changed.is_err() || *shutdown.borrow() {
                                        return;
                                    }
                                }
                            }
                        }
                    }
                    Err(error) => mark_pose_unavailable(&latest, format!("watch pose: {error}")),
                }
            }
            Err(error) => mark_pose_unavailable(&latest, format!("connect pose: {error}")),
        }
        if wait_or_shutdown(Duration::from_millis(500), &mut shutdown).await {
            return;
        }
    }
}

fn validate_pose_sample(
    sample: bello_pose_api::WatchPoseResponse,
    expected_reference: &str,
    expected_tracked: &str,
    last_sequence: Option<u64>,
) -> Result<(u64, Transform, Duration), String> {
    if sample.reference_frame != expected_reference || sample.tracked_frame != expected_tracked {
        return Err(format!(
            "pose frame mismatch: received {}_from_{}, expected {}_from_{}",
            sample.reference_frame, sample.tracked_frame, expected_reference, expected_tracked
        ));
    }
    if last_sequence.is_some_and(|last| sample.sequence <= last) {
        return Err(format!(
            "pose sequence {} is not newer than {last_sequence:?}",
            sample.sequence
        ));
    }
    let translation = sample
        .translation_metres
        .ok_or_else(|| "pose sample has no translation".to_owned())?;
    let orientation = sample
        .orientation_xyzw
        .ok_or_else(|| "pose sample has no orientation".to_owned())?;
    let norm = (orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w)
        .sqrt();
    if !norm.is_finite() || (norm - 1.0).abs() > 1.0e-6 {
        return Err(format!("pose orientation is not normalized (norm {norm})"));
    }
    let transform = Transform {
        translation: [translation.x, translation.y, translation.z],
        orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
    }
    .checked("pose sample")
    .map_err(|error| error.to_string())?;
    Ok((
        sample.sequence,
        transform,
        Duration::from_millis(sample.sample_age_ms),
    ))
}

async fn connect_pose(endpoint: &str, tls: Option<&TlsConfig>) -> anyhow::Result<Channel> {
    let mut endpoint = Endpoint::from_shared(endpoint.to_owned())?;
    if endpoint.uri().scheme_str() == Some("https") {
        let mut settings = ClientTlsConfig::new()
            .with_native_roots()
            .assume_http2(true);
        if let Some(tls) = tls {
            if let Some(path) = &tls.ca_certificate {
                settings = settings.ca_certificate(Certificate::from_pem(fs::read(path)?));
            }
            if let (Some(certificate), Some(key)) =
                (&tls.client_certificate, &tls.client_private_key)
            {
                settings =
                    settings.identity(Identity::from_pem(fs::read(certificate)?, fs::read(key)?));
            }
        }
        endpoint = endpoint.tls_config(settings)?;
    } else if tls.is_some() {
        anyhow::bail!("pose TLS settings require an https:// endpoint");
    }
    Ok(endpoint.connect().await?)
}

fn mark_pose_unavailable(latest: &RwLock<DisplaySnapshot>, warning: impl Into<String>) {
    let display = &mut *latest.write();
    display.pose_available = false;
    display.pose_warning = Some(warning.into());
}

async fn wait_or_shutdown(duration: Duration, shutdown: &mut watch::Receiver<bool>) -> bool {
    tokio::select! {
        _ = tokio::time::sleep(duration) => false,
        changed = shutdown.changed() => changed.is_err() || *shutdown.borrow(),
    }
}

#[cfg(test)]
mod tests {
    use std::pin::Pin;

    use bello_pose_api::pose_service_server::{PoseService, PoseServiceServer};
    use bello_pose_api::{
        AngularVelocity, LinearVelocity, Orientation, Translation, WatchPoseResponse,
    };
    use futures_core::Stream;
    use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
    use tonic::{Request, Response, Status};

    use super::*;

    #[derive(Default)]
    struct FakePoseService;

    #[tonic::async_trait]
    impl PoseService for FakePoseService {
        type WatchPoseStream =
            Pin<Box<dyn Stream<Item = Result<WatchPoseResponse, Status>> + Send + 'static>>;

        async fn watch_pose(
            &self,
            request: Request<WatchPoseRequest>,
        ) -> Result<Response<Self::WatchPoseStream>, Status> {
            let request = request.into_inner();
            let (sender, receiver) = tokio::sync::mpsc::channel(1);
            tokio::spawn(async move {
                let _ = sender
                    .send(Ok(WatchPoseResponse {
                        reference_frame: request.reference_frame,
                        tracked_frame: request.tracked_frame,
                        sequence: 1,
                        translation_metres: Some(Translation {
                            x: 1.0,
                            y: 2.0,
                            z: 3.0,
                        }),
                        orientation_xyzw: Some(Orientation {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 1.0,
                        }),
                        sample_age_ms: 0,
                        linear_velocity_metres_per_second: Some(LinearVelocity::default()),
                        sample_monotonic_ns: 1,
                        angular_velocity_radians_per_second: Some(AngularVelocity::default()),
                    }))
                    .await;
                tokio::time::sleep(Duration::from_secs(1)).await;
            });
            Ok(Response::new(Box::pin(ReceiverStream::new(receiver))))
        }
    }

    #[tokio::test]
    async fn pose_stream_updates_then_freezes_a_stale_root() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let (server_shutdown, server_shutdown_receiver) = tokio::sync::oneshot::channel();
        let server = tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(PoseServiceServer::new(FakePoseService))
                .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async move {
                    let _ = server_shutdown_receiver.await;
                }),
        );

        let latest = Arc::new(RwLock::new(DisplaySnapshot::default()));
        let (shutdown, receiver) = watch::channel(false);
        let monitor = tokio::spawn(monitor_pose(
            RootConfig::ExternalPose {
                endpoint: format!("http://{address}"),
                tls: None,
                reference_frame: "world".into(),
                tracked_frame: "tracker".into(),
                tracked_from_model_root: Transform {
                    translation: [0.5, 0.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                staleness_ms: 50,
            },
            Arc::clone(&latest),
            receiver,
        ));

        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if latest.read().root_transform.translation == [1.5, 2.0, 3.0] {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        let frozen = latest.read().root_transform;
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if !latest.read().pose_available {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(latest.read().root_transform, frozen);

        let _ = shutdown.send(true);
        monitor.await.unwrap();
        let _ = server_shutdown.send(());
        server.await.unwrap().unwrap();
    }

    #[test]
    fn pose_samples_reject_wrong_frames_sequences_and_non_unit_quaternions() {
        let sample = WatchPoseResponse {
            reference_frame: "world".into(),
            tracked_frame: "tracker".into(),
            sequence: 2,
            translation_metres: Some(Translation::default()),
            orientation_xyzw: Some(Orientation {
                w: 1.0,
                ..Default::default()
            }),
            sample_age_ms: 3,
            linear_velocity_metres_per_second: Some(LinearVelocity::default()),
            sample_monotonic_ns: 1,
            angular_velocity_radians_per_second: Some(AngularVelocity::default()),
        };
        assert!(validate_pose_sample(sample.clone(), "map", "tracker", None).is_err());
        assert!(validate_pose_sample(sample.clone(), "world", "tracker", Some(2)).is_err());
        let mut invalid = sample;
        invalid.orientation_xyzw.as_mut().unwrap().w = 2.0;
        assert!(validate_pose_sample(invalid, "world", "tracker", None).is_err());
    }

    #[test]
    fn diagnostic_events_collapse_repeats_and_remain_bounded() {
        let mut raw = RawHardware::default();
        push_event(&mut raw, protocol_event(1, "same".into()));
        push_event(&mut raw, protocol_event(2, "same".into()));
        assert_eq!(raw.events.len(), 1);
        assert_eq!(raw.events[0].repeat_count, 2);
        assert_eq!(raw.events[0].monotonic_ns, Some(2));
        let cleared_through = raw.events[0].id;
        push_event(&mut raw, protocol_event(3, "different".into()));
        push_event(&mut raw, protocol_event(4, "same".into()));
        assert_eq!(raw.events.len(), 2);
        let repeated = raw.events.back().unwrap();
        assert_eq!(repeated.detail, "same");
        assert_eq!(repeated.repeat_count, 3);
        assert!(repeated.id > cleared_through);

        for index in 0..=DIAGNOSTIC_EVENT_CAPACITY {
            push_event(
                &mut raw,
                protocol_event(index as u64 + 3, format!("event {index}")),
            );
        }
        assert_eq!(raw.events.len(), DIAGNOSTIC_EVENT_CAPACITY);
        assert!(raw.events_evicted > 0);
        assert!(raw.events.iter().all(|event| event.id > 0));
    }

    #[test]
    fn lifecycle_health_uses_exact_inventory_component_names() {
        let mut raw = RawHardware::default();
        raw.buses.insert(
            "body/can".into(),
            DisplayBus {
                name: "body/can".into(),
                transport: "socketcan:can0".into(),
                host_id: 1,
                status: None,
            },
        );
        let status = DisplayDeviceStatus {
            state: DeviceState::Online,
            detail: "online".into(),
            transition_count: 2,
        };
        apply_component_status(&mut raw, "can/body/can", Some(status));
        assert_eq!(
            raw.buses["body/can"]
                .status
                .as_ref()
                .map(|status| status.state),
            Some(DeviceState::Online)
        );
    }
}

#[cfg(test)]
mod hardware_tests;
