//! Read-only camera commissioning fixture: no CAN, controller or actuator commands.
use anyhow::Result;
use bello_sim::{cameras, model, runtime::RunConfig};
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
        /// Preview camera outputs from a paired emulator profile.
        #[arg(long, conflicts_with_all = ["model_config", "camera_config"], required_unless_present = "model_config")]
        config: Option<PathBuf>,
        #[arg(long, requires = "camera_config")]
        model_config: Option<PathBuf>,
        #[arg(long, requires = "model_config")]
        camera_config: Option<PathBuf>,
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
            config,
            model_config,
            camera_config,
            seconds,
        } => {
            anyhow::ensure!(seconds.is_finite() && seconds > 0., "invalid duration");
            let (model, outputs) = match (config, model_config, camera_config) {
                (Some(path), None, None) => {
                    let paired = RunConfig::load(&path, None)?;
                    (paired.model, paired.camera_outputs)
                }
                (None, Some(model), Some(camera)) => {
                    let model = model::ModelConfig::load(&model)?;
                    let output = cameras::Config::load(&camera, &model.hardware)?;
                    (model, vec![output])
                }
                _ => anyhow::bail!("specify a paired profile or a model and camera profile"),
            };
            anyhow::ensure!(
                !outputs.is_empty(),
                "no simulated cameras in the deployment"
            );
            let robot = model::build(&model, None)?;
            let mut captures = outputs
                .into_iter()
                .map(|output| cameras::Capture::start(output, &model, &robot.simulation))
                .collect::<Result<Vec<_>>>()?;
            let start = Instant::now();
            while start.elapsed().as_secs_f64() < seconds {
                for capture in &mut captures {
                    capture.check_running()?;
                    capture.sample(&robot.simulation);
                }
                std::thread::sleep(Duration::from_micros(1000));
            }
            captures
                .into_iter()
                .try_for_each(|capture| capture.finish_preview())
        }
    }
}
