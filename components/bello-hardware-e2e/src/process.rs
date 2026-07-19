use std::ffi::OsStr;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use nix::sys::signal::{Signal, kill};
use nix::unistd::Pid;

pub struct ManagedProcess {
    name: &'static str,
    child: Child,
}

impl ManagedProcess {
    pub fn spawn(
        name: &'static str,
        executable: &Path,
        arguments: impl IntoIterator<Item = impl AsRef<OsStr>>,
    ) -> Result<Self> {
        let child = Command::new(executable)
            .args(arguments)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| format!("starting {name} from {}", executable.display()))?;
        Ok(Self { name, child })
    }

    pub fn ensure_running(&mut self) -> Result<()> {
        if let Some(status) = self
            .child
            .try_wait()
            .with_context(|| format!("checking {} process", self.name))?
        {
            bail!("{} exited early with {status}", self.name);
        }
        Ok(())
    }

    fn stop(&mut self) {
        let Ok(None) = self.child.try_wait() else {
            return;
        };
        let _ = kill(Pid::from_raw(self.child.id() as i32), Signal::SIGINT);
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match self.child.try_wait() {
                Ok(Some(_)) => return,
                Ok(None) => thread::sleep(Duration::from_millis(10)),
                Err(_) => break,
            }
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        self.stop();
    }
}
