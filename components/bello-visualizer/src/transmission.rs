use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use nalgebra::{DMatrix, DVector};
use thiserror::Error;

use crate::RobotDescription;

#[derive(Debug, Error, PartialEq)]
pub enum TransmissionError {
    #[error("coordinate {0:?} is not a URDF scalar joint or differential actuator")]
    UnknownCoordinate(String),
    #[error("coordinate {0:?} appears more than once")]
    DuplicateCoordinate(String),
    #[error(
        "coordinates {coordinates:?} underdetermine model joints {joints:?}; transmission rank is {rank}, expected {expected}"
    )]
    Underdetermined {
        coordinates: Vec<String>,
        joints: Vec<String>,
        rank: usize,
        expected: usize,
    },
    #[error("mapping needs {} values, got {actual}", .expected)]
    Dimension { expected: usize, actual: usize },
}

/// Precomputed linear mapping `coordinate = forward * model_joint`.
#[derive(Clone, Debug)]
pub struct Transmission {
    coordinate_names: Vec<String>,
    joint_names: Vec<String>,
    forward: DMatrix<f64>,
    inverse: DMatrix<f64>,
    model_effort_to_coordinate: DMatrix<f64>,
    coordinate_offset: DVector<f64>,
}

impl Transmission {
    pub fn from_description(
        description: &RobotDescription,
        coordinates: impl IntoIterator<Item = impl AsRef<str>>,
    ) -> Result<Self, TransmissionError> {
        let coordinate_names = coordinates
            .into_iter()
            .map(|name| name.as_ref().to_owned())
            .collect::<Vec<_>>();
        let mut unique = BTreeSet::new();
        for name in &coordinate_names {
            if !unique.insert(name.clone()) {
                return Err(TransmissionError::DuplicateCoordinate(name.clone()));
            }
        }

        let mut rows = Vec::<BTreeMap<String, f64>>::new();
        let mut offsets = Vec::new();
        let mut affected = BTreeSet::new();
        for coordinate in &coordinate_names {
            let (row, offset) = if let Some(joint) = description.joint(coordinate) {
                if joint.kind == bello_robot_description::JointKind::Fixed {
                    return Err(TransmissionError::UnknownCoordinate(coordinate.clone()));
                }
                (BTreeMap::from([(coordinate.clone(), 1.0)]), 0.0)
            } else if let Some(transmission) = description.transmission_for_coordinate(coordinate) {
                let coordinate_index = transmission
                    .coordinates
                    .iter()
                    .position(|name| name == coordinate)
                    .expect("matching transmission contains coordinate");
                let sign = if coordinate_index == 0 { 1.0 } else { -1.0 };
                let coordinate_reduction = transmission.coordinate_reduction[coordinate_index];
                (
                    BTreeMap::from([
                        (
                            transmission.joints[0].clone(),
                            coordinate_reduction * transmission.joint_reduction[0],
                        ),
                        (
                            transmission.joints[1].clone(),
                            sign * coordinate_reduction * transmission.joint_reduction[1],
                        ),
                    ]),
                    -coordinate_reduction
                        * (transmission.joint_reduction[0] * transmission.joint_offset[0]
                            + sign
                                * transmission.joint_reduction[1]
                                * transmission.joint_offset[1]),
                )
            } else {
                return Err(TransmissionError::UnknownCoordinate(coordinate.clone()));
            };
            affected.extend(row.keys().cloned());
            rows.push(row);
            offsets.push(offset);
        }
        let joint_names = affected.into_iter().collect::<Vec<_>>();
        let joint_indices = joint_names
            .iter()
            .enumerate()
            .map(|(index, name)| (name.as_str(), index))
            .collect::<BTreeMap<_, _>>();
        let mut forward = DMatrix::zeros(coordinate_names.len(), joint_names.len());
        for (row_index, row) in rows.iter().enumerate() {
            for (joint, coefficient) in row {
                forward[(row_index, joint_indices[joint.as_str()])] = *coefficient;
            }
        }

        let singular = forward.clone().svd(false, false).singular_values;
        let largest = singular.iter().copied().fold(0.0_f64, f64::max);
        let tolerance =
            largest * (forward.nrows().max(forward.ncols()) as f64) * f64::EPSILON * 100.0;
        let rank = singular.iter().filter(|value| **value > tolerance).count();
        if rank < joint_names.len() {
            return Err(TransmissionError::Underdetermined {
                coordinates: coordinate_names,
                joints: joint_names,
                rank,
                expected: forward.ncols(),
            });
        }
        let gram = forward.transpose() * &forward;
        let gram_inverse =
            gram.try_inverse()
                .ok_or_else(|| TransmissionError::Underdetermined {
                    coordinates: coordinate_names.clone(),
                    joints: joint_names.clone(),
                    rank,
                    expected: forward.ncols(),
                })?;
        let inverse = &gram_inverse * forward.transpose();
        let model_effort_to_coordinate = &forward * gram_inverse;
        Ok(Self {
            coordinate_names,
            joint_names,
            forward,
            inverse,
            model_effort_to_coordinate,
            coordinate_offset: DVector::from_vec(offsets),
        })
    }

    pub fn coordinate_names(&self) -> &[String] {
        &self.coordinate_names
    }

    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    pub fn forward_matrix(&self) -> &DMatrix<f64> {
        &self.forward
    }

    pub(crate) fn model_effort_to_coordinate_matrix(&self) -> &DMatrix<f64> {
        &self.model_effort_to_coordinate
    }

    pub fn joint_to_coordinate(&self, values: &[f64]) -> Result<Vec<f64>, TransmissionError> {
        let values = self.multiply_vector(&self.forward, values)?;
        Ok((values + &self.coordinate_offset).iter().copied().collect())
    }

    pub fn coordinate_to_joint(&self, values: &[f64]) -> Result<Vec<f64>, TransmissionError> {
        if values.len() != self.coordinate_offset.len() {
            return Err(TransmissionError::Dimension {
                expected: self.coordinate_offset.len(),
                actual: values.len(),
            });
        }
        let centered = DVector::from_column_slice(values) - &self.coordinate_offset;
        Ok((&self.inverse * centered).iter().copied().collect())
    }

    pub fn joint_velocity_to_coordinate(
        &self,
        values: &[f64],
    ) -> Result<Vec<f64>, TransmissionError> {
        self.multiply(&self.forward, values)
    }

    pub fn coordinate_velocity_to_joint(
        &self,
        values: &[f64],
    ) -> Result<Vec<f64>, TransmissionError> {
        self.multiply(&self.inverse, values)
    }

    /// Maps actuator effort to generalized model-joint effort by virtual work.
    pub fn coordinate_effort_to_joint(
        &self,
        values: &[f64],
    ) -> Result<Vec<f64>, TransmissionError> {
        self.multiply(&self.forward.transpose(), values)
    }

    /// Minimum-norm actuator effort that realizes the requested model-joint effort.
    pub fn joint_effort_to_coordinate(
        &self,
        values: &[f64],
    ) -> Result<Vec<f64>, TransmissionError> {
        self.multiply(&self.model_effort_to_coordinate, values)
    }

    fn multiply(
        &self,
        matrix: &DMatrix<f64>,
        values: &[f64],
    ) -> Result<Vec<f64>, TransmissionError> {
        Ok(self
            .multiply_vector(matrix, values)?
            .iter()
            .copied()
            .collect())
    }

    fn multiply_vector(
        &self,
        matrix: &DMatrix<f64>,
        values: &[f64],
    ) -> Result<DVector<f64>, TransmissionError> {
        if matrix.ncols() != values.len() {
            return Err(TransmissionError::Dimension {
                expected: matrix.ncols(),
                actual: values.len(),
            });
        }
        Ok(matrix * DVector::from_column_slice(values))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateFeedback {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub age: Duration,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointFeedback {
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
    pub available: bool,
}

/// Last-valid reconstruction used when one or more required coordinates are absent or stale.
pub struct FeedbackReconstructor {
    transmission: Transmission,
    maximum_age: Duration,
    last: JointFeedback,
}

impl FeedbackReconstructor {
    pub fn new(transmission: Transmission, maximum_age: Duration) -> Self {
        let joint_count = transmission.joint_names.len();
        Self {
            transmission,
            maximum_age,
            last: JointFeedback {
                position: vec![0.0; joint_count],
                velocity: vec![0.0; joint_count],
                effort: vec![0.0; joint_count],
                available: false,
            },
        }
    }

    pub fn update(&mut self, feedback: &BTreeMap<String, CoordinateFeedback>) -> &JointFeedback {
        let complete = self
            .transmission
            .coordinate_names
            .iter()
            .map(|name| feedback.get(name))
            .collect::<Option<Vec<_>>>();
        let Some(complete) = complete else {
            self.last.available = false;
            return &self.last;
        };
        if complete.iter().any(|sample| {
            sample.age > self.maximum_age
                || ![sample.position, sample.velocity, sample.effort]
                    .iter()
                    .all(|value| value.is_finite())
        }) {
            self.last.available = false;
            return &self.last;
        }
        let position = complete
            .iter()
            .map(|sample| sample.position)
            .collect::<Vec<_>>();
        let velocity = complete
            .iter()
            .map(|sample| sample.velocity)
            .collect::<Vec<_>>();
        let effort = complete
            .iter()
            .map(|sample| sample.effort)
            .collect::<Vec<_>>();
        self.last.position = self
            .transmission
            .coordinate_to_joint(&position)
            .expect("complete position vector has the precomputed dimension");
        self.last.velocity = self
            .transmission
            .coordinate_velocity_to_joint(&velocity)
            .expect("complete velocity vector has the precomputed dimension");
        self.last.effort = self
            .transmission
            .coordinate_effort_to_joint(&effort)
            .expect("complete effort vector has the precomputed dimension");
        self.last.available = true;
        &self.last
    }

    pub fn transmission(&self) -> &Transmission {
        &self.transmission
    }
}
