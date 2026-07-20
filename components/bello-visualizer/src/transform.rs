use serde::{Deserialize, Serialize};

use crate::mjcf::ModelError;

/// Rigid transform with translation in metres and quaternion in xyzw order.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Transform {
    #[serde(default)]
    pub translation: [f64; 3],
    #[serde(default = "identity_xyzw")]
    pub orientation_xyzw: [f64; 4],
}

const fn identity_xyzw() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        orientation_xyzw: identity_xyzw(),
    };

    pub fn checked(self, context: &str) -> Result<Self, ModelError> {
        if !self
            .translation
            .iter()
            .chain(self.orientation_xyzw.iter())
            .all(|value| value.is_finite())
        {
            return Err(ModelError::InvalidTransform(format!(
                "{context} contains a non-finite value"
            )));
        }
        let norm = self
            .orientation_xyzw
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        if norm <= 1.0e-12 {
            return Err(ModelError::InvalidTransform(format!(
                "{context} has a zero-length quaternion"
            )));
        }
        let mut normalized = self;
        for value in &mut normalized.orientation_xyzw {
            *value /= norm;
        }
        Ok(normalized)
    }

    /// Compose `self_from_parent * parent_from_child`.
    pub fn compose(self, child: Self) -> Self {
        let rotated = rotate(self.orientation_xyzw, child.translation);
        Self {
            translation: [
                self.translation[0] + rotated[0],
                self.translation[1] + rotated[1],
                self.translation[2] + rotated[2],
            ],
            orientation_xyzw: multiply(self.orientation_xyzw, child.orientation_xyzw),
        }
    }
}

pub(crate) fn wxyz_to_xyzw(values: [f64; 4]) -> [f64; 4] {
    [values[1], values[2], values[3], values[0]]
}

fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (ax, ay, az, aw) = (a[0], a[1], a[2], a[3]);
    let (bx, by, bz, bw) = (b[0], b[1], b[2], b[3]);
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [v[0], v[1], v[2], 0.0];
    let conjugate = [-q[0], -q[1], -q[2], q[3]];
    let result = multiply(multiply(q, qv), conjugate);
    [result[0], result[1], result[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_mujoco_wxyz_to_renderer_xyzw() {
        assert_eq!(wxyz_to_xyzw([1.0, 2.0, 3.0, 4.0]), [2.0, 3.0, 4.0, 1.0]);
    }

    #[test]
    fn composes_translation_through_rotation() {
        let half = 0.5_f64.sqrt();
        let parent = Transform {
            translation: [1.0, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, half, half],
        };
        let result = parent.compose(Transform {
            translation: [1.0, 0.0, 0.0],
            ..Transform::IDENTITY
        });
        assert!((result.translation[0] - 1.0).abs() < 1.0e-12);
        assert!((result.translation[1] - 1.0).abs() < 1.0e-12);
    }
}
