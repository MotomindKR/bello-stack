use std::net::{SocketAddr, TcpListener as StdTcpListener};
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Context;
use bello_remote_api::remote_command_service_server::{
    RemoteCommandService, RemoteCommandServiceServer,
};
use bello_remote_api::{WatchCommandRequest, WatchCommandResponse};
use futures_core::Stream;
use gilrs::{Axis, Button, Gilrs};
use parking_lot::{Mutex, RwLock};
use tokio::net::TcpListener;
use tokio::sync::watch;
use tokio_stream::wrappers::TcpListenerStream;
use tonic::{Request, Response, Status};

use crate::config::RemoteConfig;
use crate::runtime::{DisplayRemote, DisplaySnapshot};

#[derive(Clone, Debug, PartialEq)]
pub struct RemoteSample {
    pub sequence: u64,
    pub sample_monotonic_ns: u64,
    pub controller_connected: bool,
    pub controller_name: String,
    pub intent_armed: bool,
    pub normalized_forward: f64,
    pub normalized_lateral: f64,
    pub normalized_yaw: f64,
    pub abort_active: bool,
}

impl RemoteSample {
    fn disconnected(sequence: u64, sample_monotonic_ns: u64) -> Self {
        Self {
            sequence,
            sample_monotonic_ns,
            controller_connected: false,
            controller_name: String::new(),
            intent_armed: false,
            normalized_forward: 0.0,
            normalized_lateral: 0.0,
            normalized_yaw: 0.0,
            abort_active: false,
        }
    }
}

impl From<RemoteSample> for WatchCommandResponse {
    fn from(sample: RemoteSample) -> Self {
        Self {
            sequence: sample.sequence,
            sample_monotonic_ns: sample.sample_monotonic_ns,
            controller_connected: sample.controller_connected,
            controller_name: sample.controller_name,
            intent_armed: sample.intent_armed,
            normalized_forward: sample.normalized_forward,
            normalized_lateral: sample.normalized_lateral,
            normalized_yaw: sample.normalized_yaw,
            abort_active: sample.abort_active,
        }
    }
}

#[derive(Clone)]
struct RemoteServer {
    samples: watch::Receiver<RemoteSample>,
}

#[tonic::async_trait]
impl RemoteCommandService for RemoteServer {
    type WatchCommandStream =
        Pin<Box<dyn Stream<Item = Result<WatchCommandResponse, Status>> + Send + 'static>>;

    async fn watch_command(
        &self,
        _request: Request<WatchCommandRequest>,
    ) -> Result<Response<Self::WatchCommandStream>, Status> {
        let mut samples = self.samples.clone();
        let stream = async_stream::try_stream! {
            let mut last_sequence = None;
            loop {
                let sample = samples.borrow_and_update().clone();
                if last_sequence != Some(sample.sequence) {
                    last_sequence = Some(sample.sequence);
                    yield sample.into();
                }
                if samples.changed().await.is_err() {
                    break;
                }
            }
        };
        Ok(Response::new(Box::pin(stream)))
    }
}

#[derive(Clone, Debug)]
struct ControllerReading {
    key: String,
    name: String,
    forward: f64,
    lateral: f64,
    yaw: f64,
    abort_active: bool,
}

#[derive(Debug)]
struct RemoteNormalizer {
    deadzone: f64,
    active_controller: Option<String>,
    intent_armed: bool,
    sequence: u64,
    started: Instant,
}

impl RemoteNormalizer {
    fn new(deadzone: f64) -> Self {
        Self {
            deadzone,
            active_controller: None,
            intent_armed: false,
            sequence: 0,
            started: Instant::now(),
        }
    }

    fn sample(&mut self, reading: Option<ControllerReading>) -> RemoteSample {
        self.sequence = self.sequence.wrapping_add(1);
        let source_ns = self.started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let Some(reading) = reading else {
            self.active_controller = None;
            self.intent_armed = false;
            return RemoteSample::disconnected(self.sequence, source_ns);
        };

        if self.active_controller.as_deref() != Some(reading.key.as_str()) {
            self.active_controller = Some(reading.key.clone());
            self.intent_armed = false;
        }
        let forward = apply_deadzone(reading.forward, self.deadzone);
        let lateral = apply_deadzone(reading.lateral, self.deadzone);
        let yaw = apply_deadzone(reading.yaw, self.deadzone);
        if !self.intent_armed && forward == 0.0 && lateral == 0.0 && yaw == 0.0 {
            self.intent_armed = true;
        }
        let (forward, lateral, yaw) = if self.intent_armed {
            (forward, lateral, yaw)
        } else {
            (0.0, 0.0, 0.0)
        };
        RemoteSample {
            sequence: self.sequence,
            sample_monotonic_ns: source_ns,
            controller_connected: true,
            controller_name: reading.name,
            intent_armed: self.intent_armed,
            normalized_forward: forward,
            normalized_lateral: lateral,
            normalized_yaw: yaw,
            abort_active: reading.abort_active,
        }
    }
}

pub struct RemoteRuntime {
    config: RemoteConfig,
    listen: SocketAddr,
    listener: StdTcpListener,
    gilrs: Mutex<Gilrs>,
}

impl RemoteRuntime {
    pub fn bind(config: RemoteConfig) -> anyhow::Result<Self> {
        let listener = StdTcpListener::bind(config.listen)
            .with_context(|| format!("bind remote command service to {}", config.listen))?;
        listener
            .set_nonblocking(true)
            .context("make remote command listener nonblocking")?;
        let listen = listener
            .local_addr()
            .context("resolve remote command listen address")?;
        let gilrs = Gilrs::new().map_err(|error| anyhow::anyhow!("initialize gamepad: {error}"))?;
        Ok(Self {
            config,
            listen,
            listener,
            gilrs: Mutex::new(gilrs),
        })
    }

    pub fn listen(&self) -> SocketAddr {
        self.listen
    }

    pub async fn run(
        self,
        latest: Arc<RwLock<DisplaySnapshot>>,
        mut shutdown: watch::Receiver<bool>,
    ) -> anyhow::Result<()> {
        let listener = TcpListener::from_std(self.listener)
            .context("adopt remote command listener in Tokio")?;
        let mut normalizer = RemoteNormalizer::new(self.config.deadzone);
        let initial = normalizer.sample(read_controller(
            &mut self.gilrs.lock(),
            self.config.controller_index,
        ));
        update_display(&latest, self.listen, &initial);
        let (samples, sample_receiver) = watch::channel(initial);
        let server = RemoteServer {
            samples: sample_receiver,
        };
        let mut server_shutdown = shutdown.clone();
        let server_task = tokio::spawn(async move {
            tonic::transport::Server::builder()
                .add_service(RemoteCommandServiceServer::new(server))
                .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async move {
                    loop {
                        if *server_shutdown.borrow() || server_shutdown.changed().await.is_err() {
                            break;
                        }
                    }
                })
                .await
        });

        let poll_period = Duration::from_secs_f64(1.0 / self.config.poll_hz);
        let mut ticker =
            tokio::time::interval_at(tokio::time::Instant::now() + poll_period, poll_period);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                _ = ticker.tick() => {
                    let reading = read_controller(&mut self.gilrs.lock(), self.config.controller_index);
                    let sample = normalizer.sample(reading);
                    update_display(&latest, self.listen, &sample);
                    if samples.send(sample).is_err() {
                        break;
                    }
                }
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                }
            }
        }
        drop(samples);
        match server_task.await {
            Ok(result) => result.context("serve remote command stream"),
            Err(error) => Err(anyhow::anyhow!(
                "remote command server task failed: {error}"
            )),
        }
    }
}

fn read_controller(gilrs: &mut Gilrs, controller_index: usize) -> Option<ControllerReading> {
    while gilrs.next_event().is_some() {}
    let (id, gamepad) = gilrs.gamepads().nth(controller_index)?;
    Some(ControllerReading {
        key: format!("{id:?}"),
        name: gamepad.name().to_owned(),
        forward: -f64::from(gamepad.value(Axis::LeftStickY)),
        lateral: f64::from(gamepad.value(Axis::LeftStickX)),
        yaw: -f64::from(gamepad.value(Axis::RightStickX)),
        abort_active: gamepad.is_pressed(Button::Start),
    })
}

fn apply_deadzone(value: f64, deadzone: f64) -> f64 {
    let value = value.clamp(-1.0, 1.0);
    let magnitude = value.abs();
    if magnitude <= deadzone {
        0.0
    } else {
        value.signum() * (magnitude - deadzone) / (1.0 - deadzone)
    }
}

fn update_display(latest: &RwLock<DisplaySnapshot>, listen: SocketAddr, sample: &RemoteSample) {
    let snapshot = &mut *latest.write();
    snapshot.remote = Some(DisplayRemote {
        listen,
        service_error: None,
        controller_connected: sample.controller_connected,
        controller_name: sample.controller_name.clone(),
        intent_armed: sample.intent_armed,
        normalized_forward: sample.normalized_forward,
        normalized_lateral: sample.normalized_lateral,
        normalized_yaw: sample.normalized_yaw,
        abort_active: sample.abort_active,
    });
    snapshot.generation = snapshot.generation.wrapping_add(1);
}

pub(crate) fn mark_service_failed(
    latest: &RwLock<DisplaySnapshot>,
    listen: SocketAddr,
    error: String,
) {
    let snapshot = &mut *latest.write();
    snapshot.remote = Some(DisplayRemote {
        listen,
        service_error: Some(error),
        controller_connected: false,
        controller_name: String::new(),
        intent_armed: false,
        normalized_forward: 0.0,
        normalized_lateral: 0.0,
        normalized_yaw: 0.0,
        abort_active: false,
    });
    snapshot.generation = snapshot.generation.wrapping_add(1);
}

#[cfg(test)]
mod tests {
    use super::*;
    use bello_remote_api::remote_command_service_client::RemoteCommandServiceClient;
    use tokio::sync::oneshot;

    fn reading(key: &str, axes: [f64; 3], abort_active: bool) -> ControllerReading {
        ControllerReading {
            key: key.into(),
            name: "test controller".into(),
            forward: axes[0],
            lateral: axes[1],
            yaw: axes[2],
            abort_active,
        }
    }

    #[test]
    fn deadzone_rescales_and_clamps() {
        assert_eq!(apply_deadzone(0.1, 0.1), 0.0);
        assert_eq!(apply_deadzone(-0.1, 0.1), 0.0);
        assert!((apply_deadzone(0.55, 0.1) - 0.5).abs() < 1.0e-12);
        assert_eq!(apply_deadzone(2.0, 0.1), 1.0);
    }

    #[test]
    fn new_controller_requires_neutral_and_abort_bypasses_gate() {
        let mut normalizer = RemoteNormalizer::new(0.1);
        let held = normalizer.sample(Some(reading("one", [1.0, 0.0, 0.0], true)));
        assert!(held.controller_connected);
        assert!(!held.intent_armed);
        assert_eq!(held.normalized_forward, 0.0);
        assert!(held.abort_active);

        let neutral = normalizer.sample(Some(reading("one", [0.0, 0.0, 0.0], false)));
        assert!(neutral.intent_armed);
        let moving = normalizer.sample(Some(reading("one", [0.55, 0.0, 0.0], false)));
        assert!((moving.normalized_forward - 0.5).abs() < 1.0e-12);

        let disconnected = normalizer.sample(None);
        assert!(!disconnected.controller_connected);
        let reconnected = normalizer.sample(Some(reading("one", [0.55, 0.0, 0.0], false)));
        assert!(!reconnected.intent_armed);
        assert_eq!(reconnected.normalized_forward, 0.0);
    }

    #[test]
    fn service_failure_clears_remote_intent_and_surfaces_the_error() {
        let latest = RwLock::new(DisplaySnapshot::default());
        let listen = "127.0.0.1:50054".parse().unwrap();
        mark_service_failed(&latest, listen, "transport failed".into());

        let snapshot = latest.read();
        let remote = snapshot.remote.as_ref().unwrap();
        assert_eq!(remote.listen, listen);
        assert_eq!(remote.service_error.as_deref(), Some("transport failed"));
        assert!(!remote.controller_connected);
        assert_eq!(remote.normalized_forward, 0.0);
        assert!(!remote.abort_active);
    }

    #[tokio::test]
    async fn service_sends_current_and_new_samples_to_multiple_subscribers() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let initial = RemoteSample::disconnected(1, 10);
        let (samples, receiver) = watch::channel(initial);
        let (shutdown, shutdown_receiver) = oneshot::channel();
        let server = tokio::spawn(async move {
            tonic::transport::Server::builder()
                .add_service(RemoteCommandServiceServer::new(RemoteServer {
                    samples: receiver,
                }))
                .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async move {
                    let _ = shutdown_receiver.await;
                })
                .await
        });
        let endpoint = format!("http://{address}");
        let mut first_client = RemoteCommandServiceClient::connect(endpoint.clone())
            .await
            .unwrap();
        let mut second_client = RemoteCommandServiceClient::connect(endpoint).await.unwrap();
        let mut first = first_client
            .watch_command(WatchCommandRequest {})
            .await
            .unwrap()
            .into_inner();
        let mut second = second_client
            .watch_command(WatchCommandRequest {})
            .await
            .unwrap()
            .into_inner();
        assert_eq!(first.message().await.unwrap().unwrap().sequence, 1);
        assert_eq!(second.message().await.unwrap().unwrap().sequence, 1);

        let mut moving = RemoteSample::disconnected(2, 20);
        moving.controller_connected = true;
        moving.intent_armed = true;
        moving.normalized_forward = 0.5;
        samples.send(moving).unwrap();
        assert_eq!(
            first.message().await.unwrap().unwrap().normalized_forward,
            0.5
        );
        assert_eq!(second.message().await.unwrap().unwrap().sequence, 2);

        drop(first);
        drop(second);
        drop(first_client);
        drop(second_client);
        shutdown.send(()).unwrap();
        server.await.unwrap().unwrap();
    }
}
