use std::io::{self, BufRead};
use std::path::PathBuf;

use anyhow::Result;
use bello_policy::Policy;
use clap::Parser;

#[derive(Parser)]
#[command(about = "CPU ONNX policy inference; reads one stacked observation JSON array per line")]
struct Args {
    #[arg(long)]
    policy: PathBuf,
    #[arg(long)]
    source: PathBuf,
    #[arg(long)]
    check: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut policy = Policy::load(&args.policy, &args.source)?;
    eprintln!(
        "{} {}: {} motor coordinates",
        policy.metadata.robot,
        policy.metadata.git_commit,
        policy.metadata.coordinates.len()
    );
    if args.check {
        return Ok(());
    }
    for line in io::stdin().lock().lines() {
        let observation: Vec<f32> = serde_json::from_str(&line?)?;
        println!("{}", serde_json::to_string(&policy.infer(&observation)?)?);
    }
    Ok(())
}
