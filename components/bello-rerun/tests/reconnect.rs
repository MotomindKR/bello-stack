use std::io::Read;
use std::net::SocketAddr;
use std::path::Path;
use std::pin::Pin;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use bello_hardware_api::hardware_service_server::{HardwareService, HardwareServiceServer};
use bello_hardware_api::watch_telemetry_response::Payload;
use bello_hardware_api::{
    AcquireLeaseRequest, AcquireLeaseResponse, DisableGroupRequest, DisableGroupResponse,
    EnableGroupRequest, EnableGroupResponse, GroupTelemetry, ListHardwareRequest,
    ListHardwareResponse, RecoverGroupRequest, RecoverGroupResponse, ReleaseLeaseRequest,
    ReleaseLeaseResponse, RenewLeaseRequest, RenewLeaseResponse, StartControlRequest,
    StartControlResponse, StopControlRequest, StopControlResponse, StreamCommandsRequest,
    StreamCommandsResponse, WatchTelemetryRequest, WatchTelemetryResponse,
};
use nix::sys::signal::{Signal, kill};
use nix::unistd::Pid;
use tokio::net::TcpListener;
use tokio::sync::oneshot;
use tokio_stream::wrappers::TcpListenerStream;
use tokio_stream::{Stream, StreamExt};
use tonic::transport::Server;
use tonic::{Request, Response, Status};

type TelemetryStream =
    Pin<Box<dyn Stream<Item = Result<WatchTelemetryResponse, Status>> + Send + 'static>>;
type CommandStream =
    Pin<Box<dyn Stream<Item = Result<StreamCommandsResponse, Status>> + Send + 'static>>;

const FIRST_STREAM: u8 = 0;
const UNAVAILABLE: u8 = 1;
const RESTARTED_STREAM: u8 = 2;

#[derive(Default)]
struct FakeState {
    phase: AtomicU8,
    watch_calls: AtomicUsize,
    events_delivered: AtomicUsize,
    unexpected_rpc: AtomicBool,
}

#[derive(Clone)]
struct FakeHardware {
    state: Arc<FakeState>,
}

impl FakeHardware {
    fn reject<T>(&self) -> Result<Response<T>, Status> {
        self.state.unexpected_rpc.store(true, Ordering::SeqCst);
        Err(Status::unimplemented("not available in observer test"))
    }
}

#[tonic::async_trait]
impl HardwareService for FakeHardware {
    type WatchTelemetryStream = TelemetryStream;
    type StreamCommandsStream = CommandStream;

    async fn list_hardware(
        &self,
        _request: Request<ListHardwareRequest>,
    ) -> Result<Response<ListHardwareResponse>, Status> {
        self.reject()
    }

    async fn acquire_lease(
        &self,
        _request: Request<AcquireLeaseRequest>,
    ) -> Result<Response<AcquireLeaseResponse>, Status> {
        self.reject()
    }

    async fn renew_lease(
        &self,
        _request: Request<RenewLeaseRequest>,
    ) -> Result<Response<RenewLeaseResponse>, Status> {
        self.reject()
    }

    async fn release_lease(
        &self,
        _request: Request<ReleaseLeaseRequest>,
    ) -> Result<Response<ReleaseLeaseResponse>, Status> {
        self.reject()
    }

    async fn enable_group(
        &self,
        _request: Request<EnableGroupRequest>,
    ) -> Result<Response<EnableGroupResponse>, Status> {
        self.reject()
    }

    async fn start_control(
        &self,
        _request: Request<StartControlRequest>,
    ) -> Result<Response<StartControlResponse>, Status> {
        self.reject()
    }

    async fn stop_control(
        &self,
        _request: Request<StopControlRequest>,
    ) -> Result<Response<StopControlResponse>, Status> {
        self.reject()
    }

    async fn disable_group(
        &self,
        _request: Request<DisableGroupRequest>,
    ) -> Result<Response<DisableGroupResponse>, Status> {
        self.reject()
    }

    async fn recover_group(
        &self,
        _request: Request<RecoverGroupRequest>,
    ) -> Result<Response<RecoverGroupResponse>, Status> {
        self.reject()
    }

    async fn watch_telemetry(
        &self,
        request: Request<WatchTelemetryRequest>,
    ) -> Result<Response<Self::WatchTelemetryStream>, Status> {
        self.state.watch_calls.fetch_add(1, Ordering::SeqCst);
        let request = request.into_inner();
        if !request.groups.is_empty()
            || !request.imus.is_empty()
            || !request.include_motors
            || !request.include_imu
            || !request.include_health
            || !request.include_command_policy_events
        {
            self.state.unexpected_rpc.store(true, Ordering::SeqCst);
            return Err(Status::invalid_argument("incomplete observer subscription"));
        }

        match self.state.phase.load(Ordering::SeqCst) {
            FIRST_STREAM => {
                self.state.phase.store(UNAVAILABLE, Ordering::SeqCst);
                let state = Arc::clone(&self.state);
                let stream = tokio_stream::iter([Ok(group_telemetry(100, 1))]).map(move |event| {
                    state.events_delivered.fetch_add(1, Ordering::SeqCst);
                    event
                });
                Ok(Response::new(Box::pin(stream)))
            }
            UNAVAILABLE => Err(Status::unavailable("scripted outage")),
            RESTARTED_STREAM => {
                let state = Arc::clone(&self.state);
                let stream = tokio_stream::iter([Ok(group_telemetry(5, 2))])
                    .map(move |event| {
                        state.events_delivered.fetch_add(1, Ordering::SeqCst);
                        event
                    })
                    .chain(tokio_stream::pending());
                Ok(Response::new(Box::pin(stream)))
            }
            phase => Err(Status::internal(format!("unexpected test phase {phase}"))),
        }
    }

    async fn stream_commands(
        &self,
        _request: Request<tonic::Streaming<StreamCommandsRequest>>,
    ) -> Result<Response<Self::StreamCommandsStream>, Status> {
        self.state.unexpected_rpc.store(true, Ordering::SeqCst);
        Err(Status::unimplemented("not available in observer test"))
    }
}

fn group_telemetry(monotonic_ns: u64, control_tick: u64) -> WatchTelemetryResponse {
    WatchTelemetryResponse {
        monotonic_ns,
        payload: Some(Payload::Group(GroupTelemetry {
            group: "test_group".to_owned(),
            control_tick,
            ..GroupTelemetry::default()
        })),
    }
}

fn runner_config(hardware_address: SocketAddr, output: &str) -> String {
    format!(
        concat!(
            "hardware_endpoint: http://{}\n",
            "reconnect:\n",
            "  initial_delay_ms: 20\n",
            "  maximum_delay_ms: 40\n",
            "output:\n",
            "{}\n",
        ),
        hardware_address, output
    )
}

struct RunnerProcess {
    child: Option<Child>,
}

impl RunnerProcess {
    fn spawn(config_path: &Path) -> Result<Self> {
        let child = Command::new(env!("CARGO_BIN_EXE_bello-rerun"))
            .args([
                "--config",
                config_path.to_str().context("non-UTF-8 config path")?,
            ])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .context("starting bello-rerun")?;
        Ok(Self { child: Some(child) })
    }

    fn ensure_running(&mut self) -> Result<()> {
        let status = self
            .child
            .as_mut()
            .context("bello-rerun is not running")?
            .try_wait()
            .context("checking bello-rerun")?;
        if let Some(status) = status {
            let diagnostics = self.take_diagnostics();
            bail!("bello-rerun exited early with {status}: {diagnostics}");
        }
        Ok(())
    }

    async fn signal_and_wait(&mut self, signal: Signal) -> Result<()> {
        let process_id = self
            .child
            .as_ref()
            .context("bello-rerun is not running")?
            .id();
        kill(Pid::from_raw(process_id as i32), signal)
            .with_context(|| format!("sending {signal:?} to bello-rerun"))?;

        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            let status = self
                .child
                .as_mut()
                .context("bello-rerun is not running")?
                .try_wait()
                .context("waiting for bello-rerun")?;
            if let Some(status) = status {
                if !status.success() {
                    let diagnostics = self.take_diagnostics();
                    bail!("bello-rerun exited unsuccessfully with {status}: {diagnostics}");
                }
                self.child = None;
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        bail!("bello-rerun did not stop within the shutdown deadline")
    }

    fn take_diagnostics(&mut self) -> String {
        let Some(mut child) = self.child.take() else {
            return "no process diagnostics".to_owned();
        };
        let mut diagnostics = String::new();
        if let Some(mut stderr) = child.stderr.take() {
            let _ = stderr.read_to_string(&mut diagnostics);
        }
        if diagnostics.trim().is_empty() {
            "no stderr output".to_owned()
        } else {
            diagnostics
        }
    }
}

impl Drop for RunnerProcess {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn binary_reconnects_records_and_remains_read_only() -> Result<()> {
    let listener = TcpListener::bind("127.0.0.1:0")
        .await
        .context("reserving test port")?;
    let server_address = listener.local_addr().context("reading test address")?;

    let temporary = tempfile::tempdir().context("creating temporary directory")?;
    let recording_path = temporary.path().join("telemetry.rrd");
    let config_path = temporary.path().join("bello-rerun.yaml");
    std::fs::write(
        &config_path,
        runner_config(
            server_address,
            &format!("  mode: save\n  path: {}", recording_path.display()),
        ),
    )
    .context("writing runner config")?;

    let mut runner = RunnerProcess::spawn(&config_path)?;
    tokio::time::sleep(Duration::from_millis(100)).await;
    runner.ensure_running()?;
    let initial_recording_size = std::fs::metadata(&recording_path).map_or(0, |value| value.len());

    let state = Arc::new(FakeState::default());
    let service = FakeHardware {
        state: Arc::clone(&state),
    };
    let (shutdown_tx, shutdown_rx) = oneshot::channel();
    let server = tokio::spawn(async move {
        Server::builder()
            .add_service(HardwareServiceServer::new(service))
            .serve_with_incoming_shutdown(TcpListenerStream::new(listener), async {
                let _ = shutdown_rx.await;
            })
            .await
    });

    wait_for(Duration::from_secs(3), || {
        state.watch_calls.load(Ordering::SeqCst) >= 2
    })
    .await
    .context("runner did not connect and observe the scripted outage")?;
    state.phase.store(RESTARTED_STREAM, Ordering::SeqCst);
    wait_for(Duration::from_secs(3), || {
        state.events_delivered.load(Ordering::SeqCst) >= 2
    })
    .await
    .context("runner did not reconnect after the scripted outage")?;

    tokio::time::sleep(Duration::from_millis(250)).await;
    runner.signal_and_wait(Signal::SIGINT).await?;
    let _ = shutdown_tx.send(());
    server
        .await
        .context("joining fake hardware server")?
        .context("running fake hardware server")?;

    assert!(!state.unexpected_rpc.load(Ordering::SeqCst));
    let recording_size = std::fs::metadata(&recording_path)
        .context("reading Rerun recording metadata")?
        .len();
    assert!(
        recording_size > initial_recording_size,
        "recording did not grow after delivered telemetry"
    );
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sigterm_is_bounded_without_a_rerun_listener() -> Result<()> {
    let hardware_listener = TcpListener::bind("127.0.0.1:0")
        .await
        .context("reserving fake hardware endpoint")?;
    let hardware_address = hardware_listener
        .local_addr()
        .context("reading fake hardware endpoint")?;
    let rerun_socket = std::net::TcpListener::bind("127.0.0.1:0")
        .context("reserving unavailable Rerun endpoint")?;
    let rerun_address = rerun_socket
        .local_addr()
        .context("reading unavailable Rerun endpoint")?;

    let temporary = tempfile::tempdir().context("creating temporary directory")?;
    let config_path = temporary.path().join("bello-rerun.yaml");
    std::fs::write(
        &config_path,
        runner_config(
            hardware_address,
            &format!("  mode: connect\n  address: {rerun_address}"),
        ),
    )
    .context("writing connect-mode runner config")?;

    let state = Arc::new(FakeState::default());
    state.phase.store(RESTARTED_STREAM, Ordering::SeqCst);
    let service = FakeHardware {
        state: Arc::clone(&state),
    };
    let (shutdown_tx, shutdown_rx) = oneshot::channel();
    let server = tokio::spawn(async move {
        Server::builder()
            .add_service(HardwareServiceServer::new(service))
            .serve_with_incoming_shutdown(TcpListenerStream::new(hardware_listener), async {
                let _ = shutdown_rx.await;
            })
            .await
    });

    let mut runner = RunnerProcess::spawn(&config_path)?;
    wait_for(Duration::from_secs(3), || {
        state.watch_calls.load(Ordering::SeqCst) >= 1
    })
    .await
    .context("runner did not become ready before SIGTERM")?;
    runner.ensure_running()?;
    runner.signal_and_wait(Signal::SIGTERM).await?;
    let _ = shutdown_tx.send(());
    server
        .await
        .context("joining fake hardware server")?
        .context("running fake hardware server")?;
    assert!(!state.unexpected_rpc.load(Ordering::SeqCst));
    Ok(())
}

async fn wait_for(timeout: Duration, condition: impl Fn() -> bool) -> Result<()> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if condition() {
            return Ok(());
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    bail!("condition was not met within {timeout:?}")
}
