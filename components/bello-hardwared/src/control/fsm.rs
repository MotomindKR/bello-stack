#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroupState {
    #[default]
    Idle,
    Enabled,
    Running,
    Fault,
    Shutdown,
}
impl GroupState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Enabled => "Enabled",
            Self::Running => "Running",
            Self::Fault => "Fault",
            Self::Shutdown => "Shutdown",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Transition {
    pub ok: bool,
    pub from: GroupState,
    pub to: GroupState,
    pub message: String,
}

#[derive(Clone, Debug, Default)]
pub struct GroupFsm {
    state: GroupState,
    fault_reason: String,
}
impl GroupFsm {
    pub fn state(&self) -> GroupState {
        self.state
    }
    pub fn fault_reason(&self) -> &str {
        &self.fault_reason
    }
    fn transition(&mut self, to: GroupState, message: impl Into<String>) -> Transition {
        let from = self.state;
        self.state = to;
        Transition {
            ok: true,
            from,
            to,
            message: message.into(),
        }
    }
    fn reject(&self, message: impl Into<String>) -> Transition {
        Transition {
            ok: false,
            from: self.state,
            to: self.state,
            message: message.into(),
        }
    }
    pub fn enable(&mut self, guards_ok: bool) -> Transition {
        if self.state != GroupState::Idle {
            return self.reject("enable is only allowed from Idle");
        }
        if !guards_ok {
            return self.reject("motor or bus guard failed");
        }
        self.fault_reason.clear();
        self.transition(GroupState::Enabled, "enabled")
    }
    pub fn start(&mut self, command_safe_and_fresh: bool) -> Transition {
        if self.state != GroupState::Enabled {
            return self.reject("start is only allowed from Enabled");
        }
        if !command_safe_and_fresh {
            return self.reject("initial command is missing, stale, or unsafe");
        }
        self.transition(GroupState::Running, "running")
    }
    pub fn stop(&mut self) -> Transition {
        if self.state == GroupState::Running {
            self.transition(GroupState::Enabled, "control stopped")
        } else {
            self.reject("stop is only allowed from Running")
        }
    }
    pub fn disable(&mut self) -> Transition {
        if matches!(self.state, GroupState::Enabled | GroupState::Running) {
            self.transition(GroupState::Idle, "disabled")
        } else if self.state == GroupState::Idle {
            self.transition(GroupState::Idle, "already idle")
        } else {
            self.reject("disable is only allowed from Enabled or Running")
        }
    }
    pub fn fault(&mut self, reason: impl Into<String>) -> Transition {
        if self.state == GroupState::Shutdown {
            return self.reject("fault ignored after Shutdown");
        }
        self.fault_reason = reason.into();
        if self.fault_reason.is_empty() {
            self.fault_reason = "unspecified fault".into();
        }
        let reason = self.fault_reason.clone();
        self.transition(GroupState::Fault, reason)
    }
    pub fn recover(&mut self, guards_ok: bool) -> Transition {
        if self.state != GroupState::Fault {
            return self.reject("recover is only allowed from Fault");
        }
        if !guards_ok {
            return self.reject("recovery guards failed");
        }
        self.fault_reason.clear();
        self.transition(GroupState::Idle, "recovered")
    }
    pub fn shutdown(&mut self) -> Transition {
        self.transition(GroupState::Shutdown, "shutdown")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn recovery_is_manual_and_returns_idle() {
        let mut f = GroupFsm::default();
        assert!(f.enable(true).ok);
        assert!(!f.start(false).ok);
        f.fault("lease lost");
        assert_eq!(f.state(), GroupState::Fault);
        assert!(!f.recover(false).ok);
        assert!(f.recover(true).ok);
        assert_eq!(f.state(), GroupState::Idle);
    }
}
