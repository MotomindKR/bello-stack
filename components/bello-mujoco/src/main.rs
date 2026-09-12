use anyhow::Result;
use bello_mujoco::model::{ModelConfig, build};
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    config: PathBuf,
    #[arg(long, required = true)]
    check: bool,
    #[arg(long)]
    export_physics: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let robot = build(
        &ModelConfig::load(&args.config)?,
        args.export_physics.as_deref(),
    )?;
    let model = robot.simulation.model();
    println!(
        "{}",
        serde_json::json!({"status": "validated", "root": robot.root_body,
        "motors": model.nu, "tendons": model.ntendon, "sensors": model.nsensor})
    );
    Ok(())
}
