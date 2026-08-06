use std::sync::Arc;
use std::time::Instant;

use super::conversion;
use crate::app::HardwareDaemon;
use crate::control::GroupSnapshot;
use crate::domain::{GroupName, ImuName, MotorName};
use crate::hardware::imu::ImuSample;
use crate::hardware::robstride::TimedFeedback;
use bello_hardware_api::{
    GetStateFrameRequest, GetStateFrameResponse, StateGroup, StateImu, StateMotor,
};

struct CollectedMotor {
    group: GroupName,
    motor: MotorName,
    feedback: TimedFeedback,
}

struct CollectedImu {
    name: ImuName,
    sample: ImuSample,
}

struct CollectedGroup {
    name: GroupName,
    snapshot: GroupSnapshot,
}

pub fn get(
    daemon: &Arc<HardwareDaemon>,
    request: &GetStateFrameRequest,
) -> Option<GetStateFrameResponse> {
    let groups = selected_groups(daemon, &request.groups);
    let imus = selected_imus(daemon, &request.imus);
    collect(daemon, &groups, &imus)
}

fn selected_groups(daemon: &HardwareDaemon, requested: &[String]) -> Vec<GroupName> {
    if requested.is_empty() {
        let mut groups = daemon
            .topology
            .groups()
            .map(|(name, _)| name.clone())
            .collect::<Vec<_>>();
        groups.sort();
        groups
    } else {
        requested
            .iter()
            .map(|name| GroupName::from(name.as_str()))
            .collect()
    }
}

fn selected_imus(daemon: &HardwareDaemon, requested: &[String]) -> Vec<ImuName> {
    if requested.is_empty() {
        let mut imus = daemon
            .topology
            .imus()
            .map(|(name, _)| name.clone())
            .collect::<Vec<_>>();
        imus.sort();
        imus
    } else {
        requested
            .iter()
            .map(|name| ImuName::from(name.as_str()))
            .collect()
    }
}

fn collect(
    daemon: &HardwareDaemon,
    groups: &[GroupName],
    imus: &[ImuName],
) -> Option<GetStateFrameResponse> {
    let mut motors = Vec::new();
    let mut group_states = Vec::with_capacity(groups.len());
    for group_name in groups {
        let group = daemon
            .topology
            .groups()
            .find(|(name, _)| *name == group_name)?
            .1;
        for motor in &group.motors {
            motors.push(CollectedMotor {
                group: group_name.clone(),
                motor: motor.clone(),
                feedback: daemon.motor_feedback(motor)?,
            });
        }
        group_states.push(CollectedGroup {
            name: group_name.clone(),
            snapshot: daemon.group(group_name).ok()?.snapshot.read().clone(),
        });
    }
    let imu_states = imus
        .iter()
        .map(|name| {
            Some(CollectedImu {
                name: name.clone(),
                sample: daemon.imu(name)?.latest()?,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    if group_states
        .iter()
        .any(|group| group.snapshot.updated.is_none())
    {
        return None;
    }

    let captured = Instant::now();
    let captured_monotonic_ns = daemon.telemetry.monotonic_ns_at(captured);
    let mut sample_times = Vec::with_capacity(motors.len() + imu_states.len() + groups.len());
    let motors = motors
        .into_iter()
        .map(|motor| {
            let (sample_monotonic_ns, sample_age_ns) =
                sample_time(captured, captured_monotonic_ns, motor.feedback.received);
            sample_times.push(sample_monotonic_ns);
            StateMotor {
                group: motor.group.to_string(),
                motor_name: motor.motor.to_string(),
                position: motor.feedback.value.position,
                velocity: motor.feedback.published_velocity,
                torque: motor.feedback.value.torque,
                temperature: motor.feedback.value.temperature,
                fault_bits: u32::from(motor.feedback.value.fault_bits),
                mode_status: u32::from(motor.feedback.value.mode_status),
                sample_monotonic_ns,
                sample_age_ns,
            }
        })
        .collect();
    let imus = imu_states
        .into_iter()
        .map(|imu| {
            let (sample_monotonic_ns, sample_age_ns) =
                sample_time(captured, captured_monotonic_ns, imu.sample.received);
            sample_times.push(sample_monotonic_ns);
            StateImu {
                imu_name: imu.name.to_string(),
                orientation: Some(conversion::quaternion(imu.sample.orientation_xyzw)),
                angular_velocity: Some(conversion::vector3(imu.sample.angular_velocity_xyz)),
                linear_acceleration: Some(conversion::vector3(imu.sample.linear_acceleration_xyz)),
                device_timestamp_ms: imu.sample.device_timestamp_ms,
                sample_monotonic_ns,
                sample_age_ns,
            }
        })
        .collect();
    let groups = group_states
        .into_iter()
        .map(|group| {
            let updated = group
                .snapshot
                .updated
                .expect("validated complete group snapshot");
            let (sample_monotonic_ns, sample_age_ns) =
                sample_time(captured, captured_monotonic_ns, updated);
            sample_times.push(sample_monotonic_ns);
            StateGroup {
                group: group.name.to_string(),
                state: conversion::group_state(group.snapshot.state),
                command_count: group.snapshot.command_count,
                control_tick: group.snapshot.control_tick,
                jitter_us: group.snapshot.jitter_us,
                detail: group.snapshot.detail,
                cause: conversion::group_cause(group.snapshot.cause),
                sample_monotonic_ns,
                sample_age_ns,
            }
        })
        .collect();
    let oldest = sample_times
        .iter()
        .copied()
        .min()
        .unwrap_or(captured_monotonic_ns);
    let newest = sample_times
        .iter()
        .copied()
        .max()
        .unwrap_or(captured_monotonic_ns);
    Some(GetStateFrameResponse {
        captured_monotonic_ns,
        motors,
        imus,
        groups,
        oldest_sample_age_ns: captured_monotonic_ns.saturating_sub(oldest),
        sample_span_ns: newest.saturating_sub(oldest),
    })
}

fn sample_time(captured: Instant, captured_monotonic_ns: u64, received: Instant) -> (u64, u64) {
    let age = captured.saturating_duration_since(received);
    let age_ns = age.as_nanos().min(u64::MAX as u128) as u64;
    (captured_monotonic_ns.saturating_sub(age_ns), age_ns)
}
