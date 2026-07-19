#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroupState {
    #[default]
    Idle,
    Enabled,
    Running,
    Fault,
    Shutdown,
}

#[derive(Clone, Debug)]
pub struct Transition {
    pub ok: bool,
    pub to: GroupState,
    pub message: String,
}

#[derive(Clone, Debug, Default)]
pub struct GroupFsm {
    state: GroupState,
    fault_detail: String,
}

impl GroupFsm {
    pub fn state(&self) -> GroupState {
        self.state
    }
    pub fn fault_detail(&self) -> &str {
        &self.fault_detail
    }

    fn transition(&mut self, to: GroupState, message: impl Into<String>) -> Transition {
        self.state = to;
        Transition {
            ok: true,
            to,
            message: message.into(),
        }
    }
    fn reject(&self, message: impl Into<String>) -> Transition {
        Transition {
            ok: false,
            to: self.state,
            message: message.into(),
        }
    }

    pub fn validate_enable(&self) -> Option<Transition> {
        if self.state != GroupState::Idle {
            Some(self.reject("enable is only allowed from Idle"))
        } else {
            None
        }
    }

    pub fn enable(&mut self) -> Transition {
        if let Some(rejection) = self.validate_enable() {
            return rejection;
        }
        self.fault_detail.clear();
        self.transition(GroupState::Enabled, "enabled")
    }

    pub fn validate_start(&self) -> Option<Transition> {
        if self.state != GroupState::Enabled {
            Some(self.reject("start is only allowed from Enabled"))
        } else {
            None
        }
    }

    pub fn start(&mut self) -> Transition {
        if let Some(rejection) = self.validate_start() {
            return rejection;
        }
        self.transition(GroupState::Running, "running")
    }

    pub fn validate_stop(&self) -> Option<Transition> {
        if self.state != GroupState::Running {
            Some(self.reject("stop is only allowed from Running"))
        } else {
            None
        }
    }

    pub fn stop(&mut self) -> Transition {
        if let Some(rejection) = self.validate_stop() {
            return rejection;
        }
        self.transition(GroupState::Enabled, "control stopped")
    }

    pub fn validate_disable(&self) -> Option<Transition> {
        if matches!(
            self.state,
            GroupState::Idle | GroupState::Enabled | GroupState::Running
        ) {
            None
        } else {
            Some(self.reject("disable is only allowed from Idle, Enabled, or Running"))
        }
    }

    pub fn disable(&mut self) -> Transition {
        if let Some(rejection) = self.validate_disable() {
            return rejection;
        }
        if matches!(self.state, GroupState::Enabled | GroupState::Running) {
            self.transition(GroupState::Idle, "disabled")
        } else {
            self.transition(GroupState::Idle, "already idle")
        }
    }

    pub fn validate_fault(&self) -> Option<Transition> {
        if self.state == GroupState::Shutdown {
            Some(self.reject("fault ignored after Shutdown"))
        } else {
            None
        }
    }

    pub fn fault(&mut self, reason: impl Into<String>) -> Transition {
        if let Some(rejection) = self.validate_fault() {
            return rejection;
        }
        self.fault_detail = reason.into();
        if self.fault_detail.is_empty() {
            self.fault_detail = "unspecified fault".into();
        }
        let reason = self.fault_detail.clone();
        self.transition(GroupState::Fault, reason)
    }

    pub fn validate_recover(&self) -> Option<Transition> {
        if self.state != GroupState::Fault {
            Some(self.reject("recover is only allowed from Fault"))
        } else {
            None
        }
    }

    pub fn recover(&mut self) -> Transition {
        if let Some(rejection) = self.validate_recover() {
            return rejection;
        }
        self.fault_detail.clear();
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
        assert!(f.enable().ok);
        assert!(f.start().ok);
        assert!(!f.start().ok);
        f.fault("lease lost");
        assert_eq!(f.state(), GroupState::Fault);
        assert!(f.recover().ok);
        assert_eq!(f.state(), GroupState::Idle);
    }
}
