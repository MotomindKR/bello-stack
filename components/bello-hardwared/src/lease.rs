use std::collections::HashMap;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use thiserror::Error;

use crate::domain::{GroupName, LeaseToken};

#[derive(Clone, Debug)]
pub struct Lease {
    pub group: GroupName,
    pub token: LeaseToken,
    pub client: String,
    pub expires: Instant,
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum LeaseError {
    #[error("group {group} is leased by {client}")]
    AlreadyHeld { group: GroupName, client: String },
    #[error("group {0} has no lease")]
    NoLease(GroupName),
    #[error("unknown lease token")]
    UnknownToken,
    #[error("lease token does not own this group")]
    WrongOwner,
    #[error("lease has expired")]
    Expired,
}

pub struct LeaseManager {
    leases: Mutex<HashMap<GroupName, Lease>>,
    default: Duration,
    maximum: Duration,
}

impl LeaseManager {
    pub fn new(default_ms: u64, maximum_ms: u64) -> Self {
        Self {
            leases: Mutex::new(HashMap::new()),
            default: Duration::from_millis(default_ms),
            maximum: Duration::from_millis(maximum_ms),
        }
    }

    fn duration(&self, requested_ms: u64) -> Duration {
        if requested_ms == 0 {
            self.default
        } else {
            Duration::from_millis(requested_ms).min(self.maximum)
        }
    }

    pub fn acquire(
        &self,
        group: &GroupName,
        client: &str,
        requested_ms: u64,
    ) -> Result<Lease, LeaseError> {
        let mut leases = self.leases.lock();
        let now = Instant::now();
        if let Some(existing) = leases.get(group) {
            if existing.expires <= now {
                return Err(LeaseError::Expired);
            }
            return Err(LeaseError::AlreadyHeld {
                group: group.clone(),
                client: existing.client.clone(),
            });
        }
        let lease = Lease {
            group: group.clone(),
            token: LeaseToken::new(),
            client: client.into(),
            expires: now + self.duration(requested_ms),
        };
        leases.insert(group.clone(), lease.clone());
        Ok(lease)
    }

    pub fn renew(&self, token: LeaseToken, requested_ms: u64) -> Result<Lease, LeaseError> {
        let mut leases = self.leases.lock();
        let duration = self.duration(requested_ms);
        let now = Instant::now();
        let lease = leases
            .values_mut()
            .find(|lease| lease.token == token)
            .ok_or(LeaseError::UnknownToken)?;
        if lease.expires <= now {
            return Err(LeaseError::Expired);
        }
        lease.expires = now + duration;
        Ok(lease.clone())
    }

    pub fn verify(&self, group: &GroupName, token: LeaseToken) -> Result<(), LeaseError> {
        let leases = self.leases.lock();
        let lease = leases
            .get(group)
            .ok_or_else(|| LeaseError::NoLease(group.clone()))?;
        if lease.token != token {
            return Err(LeaseError::WrongOwner);
        }
        if lease.expires <= Instant::now() {
            return Err(LeaseError::Expired);
        }
        Ok(())
    }

    pub fn release(&self, token: LeaseToken) -> Result<Lease, LeaseError> {
        let mut leases = self.leases.lock();
        let group = leases
            .iter()
            .find(|(_, lease)| lease.token == token)
            .map(|(group, _)| group.clone())
            .ok_or(LeaseError::UnknownToken)?;
        Ok(leases.remove(&group).expect("lease existed"))
    }

    pub fn expire(&self) -> Vec<Lease> {
        let now = Instant::now();
        let mut leases = self.leases.lock();
        let groups: Vec<GroupName> = leases
            .iter()
            .filter(|(_, lease)| lease.expires <= now)
            .map(|(group, _)| group.clone())
            .collect();
        groups
            .into_iter()
            .filter_map(|group| leases.remove(&group))
            .collect()
    }

    pub fn expire_group(&self, group: &GroupName) -> Option<Lease> {
        let mut leases = self.leases.lock();
        if leases
            .get(group)
            .is_some_and(|lease| lease.expires <= Instant::now())
        {
            leases.remove(group)
        } else {
            None
        }
    }

    pub fn owner(&self, group: &GroupName) -> Option<String> {
        self.leases
            .lock()
            .get(group)
            .filter(|lease| lease.expires > Instant::now())
            .map(|lease| lease.client.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leases_are_exclusive_per_group_not_bus() {
        let leases = LeaseManager::new(1000, 5000);
        let left = GroupName::from("left");
        let right = GroupName::from("right");
        assert!(leases.acquire(&left, "a", 0).is_ok());
        assert!(leases.acquire(&left, "b", 0).is_err());
        assert!(leases.acquire(&right, "b", 0).is_ok());
    }
}
