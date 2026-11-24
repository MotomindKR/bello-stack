use std::collections::BTreeMap;

use crate::domain::{BusName, ImpedanceCommand};
use crate::hardware::can::CanFrame;
use crate::hardware::robstride::MotorController;

type BusBatch = (MotorController, Vec<CanFrame>);

pub(super) fn send_damping<'a>(
    motors: impl IntoIterator<Item = (&'a MotorController, ImpedanceCommand)>,
) -> Result<(), String> {
    let mut batches = BTreeMap::<BusName, BusBatch>::new();
    for (motor, command) in motors {
        batch_for(&mut batches, motor).push(motor.control_frame(command));
    }
    deliver(batches)
}

pub(super) fn stop<'a>(
    motors: impl IntoIterator<Item = &'a MotorController>,
    clear_fault: bool,
) -> Result<(), String> {
    let mut batches = BTreeMap::<BusName, BusBatch>::new();
    for motor in motors {
        motor.stop_expecting_operating_feedback();
        batch_for(&mut batches, motor).push(motor.safety_stop_frame(clear_fault));
    }
    deliver(batches)
}

fn batch_for<'a>(
    batches: &'a mut BTreeMap<BusName, BusBatch>,
    motor: &MotorController,
) -> &'a mut Vec<CanFrame> {
    &mut batches
        .entry(motor.spec.address.bus.clone())
        .or_insert_with(|| (motor.clone(), Vec::new()))
        .1
}

fn deliver(batches: BTreeMap<BusName, BusBatch>) -> Result<(), String> {
    // Submit every bus before waiting for any one of them. A failed bus must not
    // prevent the remaining buses in a motor group from receiving the reaction.
    let mut pending = Vec::with_capacity(batches.len());
    let mut failures = Vec::new();
    for (_, (controller, frames)) in batches {
        match controller.submit_safety_batch(frames) {
            Ok(batch) => pending.push(batch),
            Err(error) => failures.push(error.to_string()),
        }
    }
    for batch in pending {
        if let Err(error) = batch.wait() {
            failures.push(error.to_string());
        }
    }
    if failures.is_empty() {
        Ok(())
    } else {
        Err(failures.join("; "))
    }
}
