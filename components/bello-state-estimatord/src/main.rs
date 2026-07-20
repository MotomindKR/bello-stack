use std::path::PathBuf;

use clap::Parser;

#[derive(Parser)]
struct Arguments {
    #[arg(long, value_name = "YAML")]
    config: PathBuf,
    /// Validate configuration and the URDF without connecting to hardware.
    #[arg(long)]
    check: bool,
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "bello_state_estimatord=info".into()),
        )
        .init();
    let arguments = Arguments::parse();
    let result = if arguments.check {
        bello_state_estimatord::service::check(&arguments.config)
    } else {
        bello_state_estimatord::service::run(&arguments.config).await
    };
    if let Err(error) = result {
        eprintln!("bello-state-estimatord: {error:#}");
        std::process::exit(1);
    }
}
