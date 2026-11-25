#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroupState {
    #[default]
    Idle,
    Damped,
    Configuring,
    Running,
    Fault,
    Shutdown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroupCause {
    #[default]
    Unspecified,
    Enable,
    Start,
    Stop,
    CommandTimeout,
    Disable,
    Recover,
    CommandLimit,
    CommandPositionStep,
    SafePosition,
    StaleFeedback,
    Temperature,
    ReportedMotorFault,
    ModeStatus,
    Communication,
    Transmission,
    LeaseLoss,
    Exit,
    Internal,
    Configuration,
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
    cause: GroupCause,
    detail: String,
}

impl GroupFsm {
    pub fn state(&self) -> GroupState {
        self.state
    }
    pub fn cause(&self) -> GroupCause {
        self.cause
    }

    pub fn detail(&self) -> &str {
        &self.detail
    }

    fn transition(
        &mut self,
        to: GroupState,
        cause: GroupCause,
        message: impl Into<String>,
    ) -> Transition {
        let message = message.into();
        self.state = to;
        self.cause = cause;
        self.detail.clone_from(&message);
        Transition {
            ok: true,
            to,
            message,
        }
    }
    fn reject(&self, message: impl Into<String>) -> Transition {
        Transition {
            ok: false,
            to: self.state,
            message: message.into(),
        }
    }

    fn acknowledge(&self, message: impl Into<String>) -> Transition {
        Transition {
            ok: true,
            to: self.state,
            message: message.into(),
        }
    }

    pub(super) fn configure(&mut self, to: GroupState, detail: String) -> Transition {
        match (self.state, to) {
            (GroupState::Idle, GroupState::Configuring)
            | (GroupState::Configuring, GroupState::Idle | GroupState::Fault) => {
                self.transition(to, GroupCause::Configuration, detail)
            }
            _ => self.reject("invalid motor configuration transition"),
        }
    }

    pub fn validate_enable(&self) -> Option<Transition> {
        if !matches!(self.state, GroupState::Idle | GroupState::Damped) {
            Some(self.reject("enable is only allowed from Idle or Damped"))
        } else {
            None
        }
    }

    pub fn enable(&mut self) -> Transition {
        if let Some(rejection) = self.validate_enable() {
            return rejection;
        }
        if self.state == GroupState::Damped {
            self.acknowledge("already damped")
        } else {
            self.transition(
                GroupState::Damped,
                GroupCause::Enable,
                "enabled with damping",
            )
        }
    }

    pub fn validate_start(&self) -> Option<Transition> {
        if self.state != GroupState::Damped {
            Some(self.reject("start is only allowed from Damped"))
        } else {
            None
        }
    }

    pub fn start(&mut self) -> Transition {
        if let Some(rejection) = self.validate_start() {
            return rejection;
        }
        self.transition(GroupState::Running, GroupCause::Start, "running")
    }

    pub fn validate_stop(&self) -> Option<Transition> {
        if !matches!(self.state, GroupState::Running | GroupState::Damped) {
            Some(self.reject("stop is only allowed from Running or Damped"))
        } else {
            None
        }
    }

    pub fn stop(&mut self) -> Transition {
        if let Some(rejection) = self.validate_stop() {
            return rejection;
        }
        if self.state == GroupState::Damped {
            self.acknowledge("already damped")
        } else {
            self.transition(
                GroupState::Damped,
                GroupCause::Stop,
                "control stopped with damping",
            )
        }
    }

    pub fn damp(&mut self, cause: GroupCause, message: impl Into<String>) -> Transition {
        if self.state == GroupState::Shutdown {
            return self.reject("damping ignored after Shutdown");
        }
        self.transition(GroupState::Damped, cause, message)
    }

    pub fn idle(&mut self, cause: GroupCause, message: impl Into<String>) -> Transition {
        if self.state == GroupState::Shutdown {
            return self.reject("idle transition ignored after Shutdown");
        }
        self.transition(GroupState::Idle, cause, message)
    }

    pub fn validate_disable(&self) -> Option<Transition> {
        if matches!(
            self.state,
            GroupState::Idle | GroupState::Damped | GroupState::Running
        ) {
            None
        } else {
            Some(self.reject("disable is only allowed from Idle, Damped, or Running"))
        }
    }

    pub fn disable(&mut self) -> Transition {
        if let Some(rejection) = self.validate_disable() {
            return rejection;
        }
        if matches!(self.state, GroupState::Damped | GroupState::Running) {
            self.transition(GroupState::Idle, GroupCause::Disable, "disabled")
        } else {
            self.transition(GroupState::Idle, GroupCause::Disable, "already idle")
        }
    }

    pub fn validate_fault(&self) -> Option<Transition> {
        if self.state == GroupState::Shutdown {
            Some(self.reject("fault ignored after Shutdown"))
        } else {
            None
        }
    }

    pub fn fault(&mut self, cause: GroupCause, reason: impl Into<String>) -> Transition {
        if let Some(rejection) = self.validate_fault() {
            return rejection;
        }
        let mut reason = reason.into();
        if reason.is_empty() {
            reason = "unspecified fault".into();
        }
        self.transition(GroupState::Fault, cause, reason)
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
        self.transition(GroupState::Idle, GroupCause::Recover, "recovered")
    }

    pub fn shutdown(&mut self) -> Transition {
        self.transition(GroupState::Shutdown, GroupCause::Exit, "shutdown")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_control_passes_through_damped() {
        let mut fsm = GroupFsm::default();

        assert!(fsm.enable().ok);
        assert_eq!(fsm.state(), GroupState::Damped);
        assert_eq!(fsm.cause(), GroupCause::Enable);
        assert!(fsm.start().ok);
        assert_eq!(fsm.state(), GroupState::Running);
        assert!(fsm.stop().ok);
        assert_eq!(fsm.state(), GroupState::Damped);
        assert_eq!(fsm.cause(), GroupCause::Stop);
        assert!(fsm.disable().ok);
        assert_eq!(fsm.state(), GroupState::Idle);
    }

    #[test]
    fn damped_idempotent_operations_preserve_the_trigger() {
        let mut fsm = GroupFsm::default();
        fsm.enable();
        fsm.damp(GroupCause::LeaseLoss, "lease lost; damping retained");

        assert!(fsm.enable().ok);
        assert!(fsm.stop().ok);
        assert_eq!(fsm.state(), GroupState::Damped);
        assert_eq!(fsm.cause(), GroupCause::LeaseLoss);
        assert_eq!(fsm.detail(), "lease lost; damping retained");
    }

    #[test]
    fn hard_fault_recovery_is_manual_and_returns_idle() {
        let mut fsm = GroupFsm::default();
        fsm.enable();
        fsm.fault(GroupCause::StaleFeedback, "feedback stale");

        assert_eq!(fsm.state(), GroupState::Fault);
        assert!(!fsm.start().ok);
        assert!(fsm.recover().ok);
        assert_eq!(fsm.state(), GroupState::Idle);
        assert_eq!(fsm.cause(), GroupCause::Recover);
    }
}
