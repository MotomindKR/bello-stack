//! Read-only camera commissioning fixture: no CAN, controller or actuator commands.
use anyhow::Result;
use bello_sim::{cameras, model};
use clap::{Parser, Subcommand};
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    Preview {
        #[arg(long)]
        model_config: PathBuf,
        #[arg(long)]
        camera_config: PathBuf,
        #[arg(long, default_value_t = 60.)]
        seconds: f64,
    },
    Cameras {
        #[arg(long)]
        model: PathBuf,
        #[arg(long)]
        config: PathBuf,
    },
}
fn main() -> Result<()> {
    match Args::parse().command {
        Command::Cameras { model, config } => cameras::render(&model, &config),
        Command::Preview {
            model_config,
            camera_config,
            seconds,
        } => {
            anyhow::ensure!(seconds.is_finite() && seconds > 0., "invalid duration");
            let config = model::ModelConfig::load(&model_config)?;
            let robot = model::build(&config, None)?;
            let settings = cameras::Config::load(&camera_config, &config.hardware)?;
            let mut cameras = cameras::Capture::start(settings, &config, &robot.simulation)?;
            let start = Instant::now();
            while start.elapsed().as_secs_f64() < seconds {
                cameras.check_running()?;
                cameras.sample(&robot.simulation);
                std::thread::sleep(Duration::from_micros(1000));
            }
            cameras.finish_preview()
        }
    }
}
