//! Compare live TCP/UDS snapshot RPCs without acquiring leases or sending commands.
use std::error::Error;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::{Duration, Instant};

use bello_hardware_client::{Robot, proto};

#[tokio::main(worker_threads = 2)]
async fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let output = args
        .next()
        .ok_or("usage: ipc_benchmark OUTPUT.csv [paired|tcp|uds]")?;
    let mode = args.next().unwrap_or_else(|| "paired".into());
    if !["paired", "tcp", "uds"].contains(&mode.as_str()) {
        return Err("mode must be paired, tcp, or uds".into());
    }
    let clients = [
        Robot::connect("http://127.0.0.1:50051").await?,
        Robot::connect("unix:///run/user/1000/bello-hardwared.sock").await?,
    ];
    let request = proto::GetStateFrameRequest {
        groups: vec!["lower_body".into(), "upper_body".into()],
        imus: vec!["imu".into()],
    };
    for client in &clients {
        for _ in 0..100 {
            client
                .grpc_client()
                .get_state_frame(request.clone())
                .await?;
        }
    }
    let mut output = BufWriter::new(File::create(output)?);
    writeln!(
        output,
        "rate_hz,iteration,order,transport,rpc_us,start_lag_us,oldest_motor_us,imu_age_us"
    )?;
    let rates = if mode == "paired" {
        vec![(50, 1000), (100, 600), (200, 600), (500, 600)]
    } else {
        vec![(200, 1000), (500, 1000), (1000, 1000)]
    };
    for (rate, samples) in rates {
        let period = Duration::from_secs_f64(1.0 / f64::from(rate));
        let mut deadline = tokio::time::Instant::now();
        for iteration in 0..samples {
            tokio::time::sleep_until(deadline).await;
            let lag = tokio::time::Instant::now().saturating_duration_since(deadline);
            let order = if iteration % 2 == 0 { [0, 1] } else { [1, 0] };
            for (position, index) in order.into_iter().enumerate() {
                if mode != "paired" && mode != ["tcp", "uds"][index] {
                    continue;
                }
                let started = Instant::now();
                let response = clients[index]
                    .grpc_client()
                    .get_state_frame(request.clone())
                    .await?
                    .into_inner();
                let elapsed = started.elapsed();
                let oldest = response
                    .motors
                    .iter()
                    .map(|m| m.sample_age_ns)
                    .max()
                    .unwrap_or(0);
                let imu = response
                    .imus
                    .iter()
                    .map(|i| i.sample_age_ns)
                    .max()
                    .unwrap_or(0);
                writeln!(
                    output,
                    "{rate},{iteration},{position},{},{:.3},{:.3},{:.3},{:.3}",
                    ["tcp", "uds"][index],
                    elapsed.as_secs_f64() * 1e6,
                    lag.as_secs_f64() * 1e6,
                    oldest as f64 / 1000.0,
                    imu as f64 / 1000.0
                )?;
            }
            deadline += period;
        }
        output.flush()?;
        eprintln!("Completed {rate} Hz: {samples} {mode} cycles");
    }
    Ok(())
}
