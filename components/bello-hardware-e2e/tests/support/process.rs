use std::ffi::{OsStr, OsString};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use nix::sys::signal::{Signal, kill};
use nix::unistd::Pid;

pub struct ManagedProcess {
    name: &'static str,
    executable: PathBuf,
    arguments: Vec<OsString>,
    child: Option<Child>,
    suspended: bool,
}

impl ManagedProcess {
    pub fn spawn(
        name: &'static str,
        executable: impl AsRef<Path>,
        arguments: impl IntoIterator<Item = impl AsRef<OsStr>>,
    ) -> Result<Self> {
        let mut process = Self {
            name,
            executable: executable.as_ref().to_owned(),
            arguments: arguments
                .into_iter()
                .map(|argument| argument.as_ref().to_owned())
                .collect(),
            child: None,
            suspended: false,
        };
        process.restart()?;
        Ok(process)
    }

    pub fn ensure_running(&mut self) -> Result<()> {
        let child = self
            .child
            .as_mut()
            .with_context(|| format!("{} is not running", self.name))?;
        if let Some(status) = child
            .try_wait()
            .with_context(|| format!("checking {} process", self.name))?
        {
            self.child = None;
            bail!("{} exited early with {status}", self.name);
        }
        Ok(())
    }

    pub fn pid(&self) -> Result<u32> {
        self.child
            .as_ref()
            .map(std::process::Child::id)
            .with_context(|| format!("{} is not running", self.name))
    }

    pub fn restart(&mut self) -> Result<()> {
        self.stop();
        let child = Command::new(&self.executable)
            .args(&self.arguments)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| {
                format!("starting {} from {}", self.name, self.executable.display())
            })?;
        self.child = Some(child);
        self.suspended = false;
        Ok(())
    }

    pub fn suspend(&mut self) -> Result<()> {
        self.ensure_running()?;
        let child = self.child.as_ref().expect("checked running child");
        kill(Pid::from_raw(child.id() as i32), Signal::SIGSTOP)
            .with_context(|| format!("suspending {} process", self.name))?;
        self.suspended = true;
        Ok(())
    }

    pub fn resume(&mut self) -> Result<()> {
        self.ensure_running()?;
        let child = self.child.as_ref().expect("checked running child");
        kill(Pid::from_raw(child.id() as i32), Signal::SIGCONT)
            .with_context(|| format!("resuming {} process", self.name))?;
        self.suspended = false;
        Ok(())
    }

    pub fn interrupt(&mut self) -> Result<()> {
        self.ensure_running()?;
        if self.suspended {
            let child = self.child.as_ref().expect("checked running child");
            kill(Pid::from_raw(child.id() as i32), Signal::SIGCONT)
                .with_context(|| format!("resuming {} process for shutdown", self.name))?;
            self.suspended = false;
        }
        let child = self.child.as_ref().expect("checked running child");
        kill(Pid::from_raw(child.id() as i32), Signal::SIGINT)
            .with_context(|| format!("interrupting {} process", self.name))
    }

    pub fn stop(&mut self) {
        let Some(child) = self.child.as_mut() else {
            return;
        };
        if matches!(child.try_wait(), Ok(Some(_))) {
            self.child = None;
            self.suspended = false;
            return;
        }
        if self.suspended {
            let _ = kill(Pid::from_raw(child.id() as i32), Signal::SIGCONT);
            self.suspended = false;
        }
        let _ = kill(Pid::from_raw(child.id() as i32), Signal::SIGINT);
        // hardwared itself allows two seconds for gRPC draining; leave the
        // process harness enough margin to observe its bounded exit and RAII
        // cleanup before escalating to SIGKILL.
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            match child.try_wait() {
                Ok(Some(_)) => {
                    self.child = None;
                    self.suspended = false;
                    return;
                }
                Ok(None) => thread::sleep(Duration::from_millis(10)),
                Err(_) => break,
            }
        }
        let _ = child.kill();
        let _ = child.wait();
        self.child = None;
        self.suspended = false;
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        self.stop();
    }
}
