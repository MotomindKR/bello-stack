use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use thiserror::Error;

use crate::domain::{GroupName, LeaseToken};

const MINIMUM_LEASE: Duration = Duration::from_millis(20);

#[derive(Clone, Debug)]
pub struct Lease {
    pub group: GroupName,
    pub token: LeaseToken,
    pub client: String,
    pub expires: Instant,
    pub generation: u64,
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
    #[error("hardware daemon is shutting down")]
    Unavailable,
}

pub struct LeaseManager {
    leases: Mutex<HashMap<GroupName, Lease>>,
    default: Duration,
    maximum: Duration,
    next_generation: AtomicU64,
}

impl LeaseManager {
    pub fn new(default_ms: u64, maximum_ms: u64) -> Self {
        Self {
            leases: Mutex::new(HashMap::new()),
            default: Duration::from_millis(default_ms),
            maximum: Duration::from_millis(maximum_ms),
            next_generation: AtomicU64::new(1),
        }
    }

    fn duration(&self, requested_ms: u64) -> Duration {
        if requested_ms == 0 {
            self.default
        } else {
            Duration::from_millis(requested_ms).clamp(MINIMUM_LEASE, self.maximum)
        }
    }

    fn next_generation(&self) -> u64 {
        loop {
            let generation = self.next_generation.fetch_add(1, Ordering::Relaxed);
            if generation != 0 {
                return generation;
            }
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
            generation: self.next_generation(),
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

    pub fn verify(&self, group: &GroupName, token: LeaseToken) -> Result<u64, LeaseError> {
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
        Ok(lease.generation)
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

    #[test]
    fn ownership_generations_change_after_release() {
        let leases = LeaseManager::new(1000, 5000);
        let group = GroupName::from("group");
        let first = leases.acquire(&group, "first", 0).unwrap();
        assert_eq!(leases.verify(&group, first.token), Ok(first.generation));
        leases.release(first.token).unwrap();
        let second = leases.acquire(&group, "second", 0).unwrap();
        assert_ne!(first.generation, second.generation);
        assert_eq!(leases.verify(&group, second.token), Ok(second.generation));
    }

    #[test]
    fn generation_wrap_skips_reserved_zero() {
        let leases = LeaseManager::new(1000, 5000);
        leases.next_generation.store(u64::MAX, Ordering::Relaxed);
        let group = GroupName::from("group");
        let last = leases.acquire(&group, "last", 0).unwrap();
        assert_eq!(last.generation, u64::MAX);
        leases.release(last.token).unwrap();
        let wrapped = leases.acquire(&group, "wrapped", 0).unwrap();
        assert_eq!(wrapped.generation, 1);
    }

    #[test]
    fn requested_duration_is_clamped_to_supported_bounds() {
        let leases = LeaseManager::new(1000, 5000);
        assert_eq!(leases.duration(1), MINIMUM_LEASE);
        assert_eq!(leases.duration(10_000), Duration::from_secs(5));
        assert_eq!(leases.duration(0), Duration::from_secs(1));
    }
}
