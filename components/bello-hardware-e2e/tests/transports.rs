#[allow(dead_code)]
#[path = "support/config.rs"]
mod config;
#[allow(dead_code)]
#[path = "support/process.rs"]
mod process;

use std::fs;
use std::net::{SocketAddr, TcpListener};
use std::path::Path;
use std::process::Command;
use std::time::Duration;

use anyhow::{Context, Result};
use bello_hardware_client::raw::HardwareClient;
use bello_hardware_client::{ConnectOptions, TlsOptions};
use process::ManagedProcess;
use rcgen::{
    BasicConstraints, CertificateParams, DnType, ExtendedKeyUsagePurpose, IsCa, Issuer, KeyPair,
    KeyUsagePurpose,
};
use tokio::time::{Instant, sleep, timeout};

const UNAVAILABLE_CAN: &str = "bello_transport_test_missing_can";

#[tokio::test]
async fn daemon_accepts_server_authenticated_tls() -> Result<()> {
    let directory = tempfile::tempdir()?;
    let pki = TestPki::new()?;
    let paths = pki.write(directory.path())?;
    let address = available_loopback_address()?;
    let listeners = format!("    - {{ type: tcp, address: {address} }}\n");
    #[cfg(unix)]
    let socket = directory.path().join("hardwared-tls.sock");
    #[cfg(unix)]
    let listeners = format!(
        "{listeners}    - {{ type: unix, path: {}, mode: \"0600\" }}\n",
        socket.display(),
    );
    let grpc = format!(
        "  listen:\n{listeners}  tls:\n    certificate: {}\n    private_key: {}\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000",
        paths.server_certificate.display(),
        paths.server_private_key.display(),
    );
    let config = directory.path().join("hardwared-tls.yaml");
    config::write_hardwared_with_grpc(&config, UNAVAILABLE_CAN, &grpc, 250.0)?;
    let mut daemon = start_daemon(&config)?;

    let options = ConnectOptions::new(format!("https://localhost:{}", address.port()))
        .tls(TlsOptions::new().ca_certificate(pki.ca_certificate.clone()));
    let client = connect(options, &mut daemon).await?;
    client
        .inventory()
        .await
        .context("reading inventory over TLS")?;
    #[cfg(unix)]
    {
        let local = connect(
            ConnectOptions::new(format!("unix://{}", socket.display())),
            &mut daemon,
        )
        .await?;
        local
            .inventory()
            .await
            .context("Unix listener must not require TCP TLS")?;
    }
    drop(client);
    daemon.stop();
    Ok(())
}

#[tokio::test]
async fn daemon_requires_a_trusted_client_certificate_for_mtls() -> Result<()> {
    let directory = tempfile::tempdir()?;
    let pki = TestPki::new()?;
    let paths = pki.write(directory.path())?;
    let address = available_loopback_address()?;
    let grpc = format!(
        "  listen: [{{ type: tcp, address: {address} }}]\n  tls:\n    certificate: {}\n    private_key: {}\n    client_ca: {}\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000",
        paths.server_certificate.display(),
        paths.server_private_key.display(),
        paths.ca_certificate.display(),
    );
    let config = directory.path().join("hardwared-mtls.yaml");
    config::write_hardwared_with_grpc(&config, UNAVAILABLE_CAN, &grpc, 250.0)?;
    let mut daemon = start_daemon(&config)?;
    let endpoint = format!("https://localhost:{}", address.port());

    let unauthenticated = ConnectOptions::new(endpoint.clone())
        .connect_timeout(Duration::from_millis(300))
        .tls(TlsOptions::new().ca_certificate(pki.ca_certificate.clone()));
    let unauthenticated = HardwareClient::connect_with(unauthenticated).await?;
    let rejected = timeout(Duration::from_secs(1), unauthenticated.inventory()).await;
    assert!(!matches!(rejected, Ok(Ok(_))));
    drop(unauthenticated);

    let authenticated = ConnectOptions::new(endpoint).tls(
        TlsOptions::new()
            .ca_certificate(pki.ca_certificate.clone())
            .identity(
                pki.client_certificate.clone(),
                pki.client_private_key.clone(),
            ),
    );
    let client = connect(authenticated, &mut daemon).await?;
    client
        .inventory()
        .await
        .context("reading inventory over mTLS")?;
    run_python_mtls(&format!("https://localhost:{}", address.port()), &paths).await?;
    drop(client);
    daemon.stop();
    Ok(())
}

#[cfg(unix)]
#[tokio::test]
async fn daemon_accepts_unix_connections_and_owns_socket_lifecycle() -> Result<()> {
    use std::os::unix::fs::PermissionsExt;

    let directory = tempfile::tempdir()?;
    let socket = directory.path().join("hardwared.sock");
    let stale = std::os::unix::net::UnixListener::bind(&socket)?;
    drop(stale);
    assert!(socket.exists());
    let grpc = format!(
        "  listen:\n    - type: unix\n      path: {}\n      mode: \"0600\"\n  default_lease_ms: 1000\n  maximum_lease_ms: 10000",
        socket.display(),
    );
    let config = directory.path().join("hardwared-unix.yaml");
    config::write_hardwared_with_grpc(&config, UNAVAILABLE_CAN, &grpc, 250.0)?;
    let mut daemon = start_daemon(&config)?;
    let endpoint = format!("unix://{}", socket.display());

    let client = connect(ConnectOptions::new(endpoint), &mut daemon).await?;
    client
        .inventory()
        .await
        .context("reading inventory over a Unix socket")?;
    run_python_unix(&format!("unix://{}", socket.display())).await?;
    let mode = fs::metadata(&socket)?.permissions().mode() & 0o777;
    assert_eq!(mode, 0o600);

    drop(client);
    daemon.stop();
    assert!(!socket.exists(), "daemon left its Unix socket behind");
    Ok(())
}

#[cfg(unix)]
#[tokio::test]
async fn tcp_and_unix_share_inventory_and_control_leases() -> Result<()> {
    use bello_hardware_client::raw::LeaseDuration;

    let directory = tempfile::tempdir()?;
    let socket = directory.path().join("hardwared.sock");
    let address = available_loopback_address()?;
    let grpc = format!(
        "  listen:\n    - {{ type: tcp, address: {address} }}\n    - {{ type: unix, path: {}, mode: \"0600\" }}",
        socket.display(),
    );
    let config = directory.path().join("hardwared-dual.yaml");
    config::write_hardwared_with_grpc(&config, UNAVAILABLE_CAN, &grpc, 250.0)?;
    let mut daemon = start_daemon(&config)?;
    let tcp = connect(
        ConnectOptions::new(format!("http://{address}")),
        &mut daemon,
    )
    .await?;
    let unix = connect(
        ConnectOptions::new(format!("unix://{}", socket.display())),
        &mut daemon,
    )
    .await?;
    let tcp_inventory = tcp.inventory().await?;
    let unix_inventory = unix.inventory().await?;
    let addresses = |motors: Vec<bello_hardware_api::MotorInfo>| {
        motors
            .into_iter()
            .map(|motor| (motor.name, motor.address, motor.group))
            .collect::<Vec<_>>()
    };
    assert_eq!(
        addresses(tcp_inventory.motors),
        addresses(unix_inventory.motors)
    );
    let mut lease = tcp
        .acquire_lease("e2e_group", "tcp", LeaseDuration::ServerDefault)
        .await?;
    assert!(
        unix.acquire_lease("e2e_group", "unix", LeaseDuration::ServerDefault)
            .await
            .is_err()
    );
    lease.release().await?;
    let mut lease = unix
        .acquire_lease("e2e_group", "unix", LeaseDuration::ServerDefault)
        .await?;
    lease.release().await?;
    drop(tcp);
    drop(unix);
    daemon.stop();
    assert!(
        !socket.exists(),
        "dual-listener daemon left its socket behind"
    );
    Ok(())
}

#[cfg(unix)]
#[tokio::test]
async fn failed_second_listener_cleans_up_first_socket() -> Result<()> {
    let directory = tempfile::tempdir()?;
    let socket = directory.path().join("hardwared.sock");
    let occupied = TcpListener::bind("127.0.0.1:0")?;
    let address = occupied.local_addr()?;
    let grpc = format!(
        "  listen:\n    - {{ type: unix, path: {}, mode: \"0600\" }}\n    - {{ type: tcp, address: {address} }}",
        socket.display(),
    );
    let config = directory.path().join("hardwared-bind-failure.yaml");
    config::write_hardwared_with_grpc(&config, UNAVAILABLE_CAN, &grpc, 250.0)?;
    let mut daemon = start_daemon(&config)?;
    timeout(Duration::from_secs(3), async {
        while daemon.ensure_running().is_ok() {
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .context("daemon continued with only one bound endpoint")?;
    assert!(
        !socket.exists(),
        "failed startup left its Unix socket behind"
    );
    Ok(())
}

fn start_daemon(config: &Path) -> Result<ManagedProcess> {
    ManagedProcess::spawn(
        "bello-hardwared",
        env!("CARGO_BIN_EXE_hardwared-e2e-fixture"),
        [config],
    )
}

async fn connect(options: ConnectOptions, daemon: &mut ManagedProcess) -> Result<HardwareClient> {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        daemon.ensure_running()?;
        match HardwareClient::connect_with(options.clone()).await {
            Ok(client) => return Ok(client),
            Err(error) if Instant::now() < deadline => {
                let _ = error;
                sleep(Duration::from_millis(20)).await;
            }
            Err(error) => return Err(error).context("connecting to hardwared transport"),
        }
    }
}

fn available_loopback_address() -> Result<SocketAddr> {
    let listener = TcpListener::bind("127.0.0.1:0")?;
    let address = listener.local_addr()?;
    drop(listener);
    Ok(address)
}

#[cfg(unix)]
async fn run_python_unix(endpoint: &str) -> Result<()> {
    let endpoint = endpoint.to_owned();
    tokio::task::spawn_blocking(move || {
        let package = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent().expect("components parent").join("bello-hardware-python");
        let output = Command::new("python3")
            .args(["-c", "import sys; from bello_hardware import Robot; robot = Robot.connect(sys.argv[1], timeout=2.0); assert 'e2e_group' in robot.group_names; robot.close()", &endpoint])
            .env("PYTHONPATH", package.join("src"))
            .output().context("running Python Unix smoke client")?;
        if !output.status.success() {
            anyhow::bail!("Python Unix smoke client failed: {}", String::from_utf8_lossy(&output.stderr));
        }
        Ok(())
    }).await.context("joining Python Unix smoke client")?
}

async fn run_python_mtls(endpoint: &str, paths: &TestPkiPaths) -> Result<()> {
    let endpoint = endpoint.to_owned();
    let ca = paths.ca_certificate.clone();
    let certificate = paths.client_certificate.clone();
    let private_key = paths.client_private_key.clone();
    tokio::task::spawn_blocking(move || {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
        let package = manifest
            .parent()
            .expect("E2E component must have a components parent")
            .join("bello-hardware-python");
        let output = Command::new("python3")
            .arg(package.join("tests/tls_smoke.py"))
            .args([
                endpoint,
                ca.display().to_string(),
                certificate.display().to_string(),
                private_key.display().to_string(),
            ])
            .env("PYTHONPATH", package.join("src"))
            .output()
            .context("running the Python mTLS smoke client")?;
        if !output.status.success() {
            anyhow::bail!(
                "Python mTLS smoke client failed with {}: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            );
        }
        Ok(())
    })
    .await
    .context("joining the Python mTLS smoke client")?
}

struct TestPki {
    ca_certificate: Vec<u8>,
    server_certificate: Vec<u8>,
    server_private_key: Vec<u8>,
    client_certificate: Vec<u8>,
    client_private_key: Vec<u8>,
}

impl TestPki {
    fn new() -> Result<Self> {
        let mut ca_params = CertificateParams::new(Vec::<String>::new())?;
        ca_params.is_ca = IsCa::Ca(BasicConstraints::Unconstrained);
        ca_params
            .distinguished_name
            .push(DnType::CommonName, "Bello transport test CA");
        ca_params.key_usages = vec![
            KeyUsagePurpose::DigitalSignature,
            KeyUsagePurpose::KeyCertSign,
            KeyUsagePurpose::CrlSign,
        ];
        let ca_key = KeyPair::generate()?;
        let ca_certificate = ca_params.self_signed(&ca_key)?;
        let issuer = Issuer::new(ca_params, ca_key);

        let (server_certificate, server_private_key) = signed_leaf(
            &issuer,
            vec!["localhost".into()],
            ExtendedKeyUsagePurpose::ServerAuth,
        )?;
        let (client_certificate, client_private_key) =
            signed_leaf(&issuer, Vec::new(), ExtendedKeyUsagePurpose::ClientAuth)?;
        Ok(Self {
            ca_certificate: ca_certificate.pem().into_bytes(),
            server_certificate,
            server_private_key,
            client_certificate,
            client_private_key,
        })
    }

    fn write(&self, directory: &Path) -> Result<TestPkiPaths> {
        let paths = TestPkiPaths {
            ca_certificate: directory.join("ca.pem"),
            server_certificate: directory.join("server.pem"),
            server_private_key: directory.join("server-key.pem"),
            client_certificate: directory.join("client.pem"),
            client_private_key: directory.join("client-key.pem"),
        };
        fs::write(&paths.ca_certificate, &self.ca_certificate)?;
        fs::write(&paths.server_certificate, &self.server_certificate)?;
        fs::write(&paths.server_private_key, &self.server_private_key)?;
        fs::write(&paths.client_certificate, &self.client_certificate)?;
        fs::write(&paths.client_private_key, &self.client_private_key)?;
        Ok(paths)
    }
}

fn signed_leaf(
    issuer: &Issuer<'_, KeyPair>,
    names: Vec<String>,
    usage: ExtendedKeyUsagePurpose,
) -> Result<(Vec<u8>, Vec<u8>)> {
    let mut params = CertificateParams::new(names)?;
    let common_name = match usage {
        ExtendedKeyUsagePurpose::ServerAuth => "localhost",
        ExtendedKeyUsagePurpose::ClientAuth => "Bello transport test client",
        _ => "Bello transport test certificate",
    };
    params
        .distinguished_name
        .push(DnType::CommonName, common_name);
    params.use_authority_key_identifier_extension = true;
    params.key_usages = vec![KeyUsagePurpose::DigitalSignature];
    params.extended_key_usages = vec![usage];
    let key = KeyPair::generate()?;
    let certificate = params.signed_by(&key, issuer)?;
    Ok((
        certificate.pem().into_bytes(),
        key.serialize_pem().into_bytes(),
    ))
}

struct TestPkiPaths {
    ca_certificate: std::path::PathBuf,
    server_certificate: std::path::PathBuf,
    server_private_key: std::path::PathBuf,
    client_certificate: std::path::PathBuf,
    client_private_key: std::path::PathBuf,
}
