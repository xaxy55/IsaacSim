// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Transform helpers for URDF origins.
//!
//! Port of the transform section of `python/impl/urdf_utils.py`: URDF `rpy`
//! is fixed-axis roll-pitch-yaw applied as `Rz * Ry * Rx`. Matrices are
//! standard math convention (`m[row][col]`, translation in the last column).

use isaacsim_core_math::linalg::mat4_mul;

use crate::model::Origin;

/// Build a 4x4 homogeneous transform from an [`Origin`]
/// (legacy `_make_transform`).
pub fn origin_to_matrix(origin: &Origin) -> [[f64; 4]; 4] {
    let r = rpy_to_rotation_matrix(origin.rpy);
    [
        [r[0][0], r[0][1], r[0][2], origin.xyz[0]],
        [r[1][0], r[1][1], r[1][2], origin.xyz[1]],
        [r[2][0], r[2][1], r[2][2], origin.xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Convert a 4x4 homogeneous transform back to an [`Origin`]
/// (legacy `_set_origin`).
pub fn matrix_to_origin(m: &[[f64; 4]; 4]) -> Origin {
    let r = [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ];
    Origin {
        xyz: [m[0][3], m[1][3], m[2][3]],
        rpy: rotation_matrix_to_rpy(&r),
    }
}

/// Compose `parent * child` origins (legacy `_compose_origin`).
pub fn compose_origins(parent: &[[f64; 4]; 4], child: &Origin) -> Origin {
    matrix_to_origin(&mat4_mul(parent, &origin_to_matrix(child)))
}

/// Roll-pitch-yaw to a 3x3 rotation matrix, `Rz * Ry * Rx`
/// (legacy `_rpy_to_rotation_matrix`).
pub fn rpy_to_rotation_matrix(rpy: [f64; 3]) -> [[f64; 3]; 3] {
    let [roll, pitch, yaw] = rpy;
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// 3x3 rotation matrix to roll-pitch-yaw
/// (legacy `_rotation_matrix_to_rpy`, including its gimbal-lock branch).
pub fn rotation_matrix_to_rpy(r: &[[f64; 3]; 3]) -> [f64; 3] {
    let sy = -r[2][0];
    if sy.abs() >= 1.0 - 1e-12 {
        // gimbal lock
        let pitch = (std::f64::consts::FRAC_PI_2).copysign(sy);
        let roll = r[0][1].atan2(r[0][2]);
        [roll, pitch, 0.0]
    } else {
        let pitch = sy.clamp(-1.0, 1.0).asin();
        let roll = r[2][1].atan2(r[2][2]);
        let yaw = r[1][0].atan2(r[0][0]);
        [roll, pitch, yaw]
    }
}

/// Multiply two 3x3 matrices.
pub(crate) fn mat3_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in a.iter().enumerate() {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| row[k] * b[k][j]).sum();
        }
    }
    out
}

/// Transpose a 3x3 matrix.
pub(crate) fn mat3_transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in m.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            out[j][i] = *value;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rpy_round_trip() {
        let cases = [
            [0.0, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.0, 0.4, 0.0],
            [0.0, 0.0, 0.5],
            [0.3, -0.4, 0.5],
        ];
        for rpy in cases {
            let back = rotation_matrix_to_rpy(&rpy_to_rotation_matrix(rpy));
            for (a, b) in rpy.iter().zip(&back) {
                assert!((a - b).abs() < 1e-12, "{rpy:?} -> {back:?}");
            }
        }
    }

    #[test]
    fn test_compose_translation_and_rotation() {
        // 90-deg yaw then translate (1, 0, 0) in the child frame -> (0, 1, 0)
        let parent = origin_to_matrix(&Origin {
            xyz: [0.0, 0.0, 0.0],
            rpy: [0.0, 0.0, std::f64::consts::FRAC_PI_2],
        });
        let composed = compose_origins(
            &parent,
            &Origin {
                xyz: [1.0, 0.0, 0.0],
                rpy: [0.0, 0.0, 0.0],
            },
        );
        assert!((composed.xyz[0]).abs() < 1e-12);
        assert!((composed.xyz[1] - 1.0).abs() < 1e-12);
    }
}
