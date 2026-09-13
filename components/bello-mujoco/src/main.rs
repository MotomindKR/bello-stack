use anyhow::Result;
use bello_mujoco::model::{ModelConfig, build};
use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Render a native emulator recording using Linux software Mesa.
    Render {
        #[arg(long)]
        recording: PathBuf,
    },
    Check {
        #[arg(long)]
        config: PathBuf,
        #[arg(long)]
        export_physics: Option<PathBuf>,
    },
    Run {
        #[arg(long)]
        config: PathBuf,
        #[arg(long)]
        duration_seconds: Option<f64>,
    },
}

fn main() -> Result<()> {
    let (config, export_physics) = match Args::parse().command {
        Command::Render { recording } => {
            // The renderer starts here before any threads or graphics libraries.
            unsafe {
                std::env::set_var("LIBGL_ALWAYS_SOFTWARE", "true");
                std::env::set_var("GALLIUM_DRIVER", "llvmpipe");
                std::env::set_var("LP_NUM_THREADS", "2");
            }
            return bello_mujoco::video::render(&recording);
        }
        Command::Check {
            config,
            export_physics,
        } => (config, export_physics),
        Command::Run {
            config,
            duration_seconds,
        } => {
            let duration = duration_seconds
                .map(|seconds| {
                    anyhow::ensure!(
                        seconds.is_finite() && seconds > 0.,
                        "duration must be finite and positive"
                    );
                    Ok(std::time::Duration::try_from_secs_f64(seconds)?)
                })
                .transpose()?;
            return bello_mujoco::runtime::run(
                bello_mujoco::runtime::Config::load(&config)?,
                duration,
            );
        }
    };
    let robot = build(&ModelConfig::load(&config)?, export_physics.as_deref())?;
    let model = robot.simulation.model();
    println!(
        "{}",
        serde_json::json!({"status": "validated", "root": robot.root_body,
        "motors": model.nu, "tendons": model.ntendon, "sensors": model.nsensor})
    );
    Ok(())
}
