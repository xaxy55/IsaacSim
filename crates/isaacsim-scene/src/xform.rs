// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Transform composition over a [`SceneStage`].
//!
//! Computes local and local-to-world transforms from the canonical
//! `translate * orient * scale` xform-op stack (the subset the ported logic
//! layer authors; see the cloner's xform-op normalization). Matrices are
//! standard math convention (`m[row][col]`, translation in the last column,
//! `world = parent * local`), matching `isaacsim-core-math`.

use isaacsim_core_math::linalg::mat4_mul;
use isaacsim_core_math::transform::quaternion_to_rotation_matrix;

use crate::backend::SceneStage;
use crate::path::parent_path;

/// The local transform of the prim at `path` from its
/// `xformOp:translate` / `xformOp:orient` / `xformOp:scale` attributes
/// (missing ops contribute identity).
pub fn local_transform<S: SceneStage + ?Sized>(stage: &S, path: &str) -> [[f64; 4]; 4] {
    let translate = stage
        .attribute(path, "xformOp:translate")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([0.0, 0.0, 0.0]);
    let orient = stage
        .attribute(path, "xformOp:orient")
        .and_then(|v| v.as_quatd())
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let scale = stage
        .attribute(path, "xformOp:scale")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([1.0, 1.0, 1.0]);

    let r = quaternion_to_rotation_matrix(orient);
    // T * R * S in column convention
    [
        [
            r[0][0] * scale[0],
            r[0][1] * scale[1],
            r[0][2] * scale[2],
            translate[0],
        ],
        [
            r[1][0] * scale[0],
            r[1][1] * scale[1],
            r[1][2] * scale[2],
            translate[1],
        ],
        [
            r[2][0] * scale[0],
            r[2][1] * scale[1],
            r[2][2] * scale[2],
            translate[2],
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// The local-to-world transform of the prim at `path`
/// (`UsdGeom.Xformable.ComputeLocalToWorldTransform` for the TRS subset),
/// composing local transforms from the root down.
pub fn local_to_world_transform<S: SceneStage + ?Sized>(stage: &S, path: &str) -> [[f64; 4]; 4] {
    let mut chain = Vec::new();
    let mut current = Some(path);
    while let Some(p) = current {
        if p == "/" {
            break;
        }
        chain.push(p);
        current = parent_path(p);
    }
    let mut world = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    for p in chain.iter().rev() {
        world = mat4_mul(&world, &local_transform(stage, p));
    }
    world
}

/// Translation component of a transform.
pub fn extract_translation(m: &[[f64; 4]; 4]) -> [f64; 3] {
    [m[0][3], m[1][3], m[2][3]]
}

/// Rotation component of a transform as a quaternion `[w, x, y, z]`,
/// normalizing out per-axis scale (assumes no shear).
pub fn extract_rotation_quaternion(m: &[[f64; 4]; 4]) -> [f64; 4] {
    let mut r = [[0.0; 3]; 3];
    for col in 0..3 {
        let norm =
            (m[0][col] * m[0][col] + m[1][col] * m[1][col] + m[2][col] * m[2][col]).sqrt();
        let inv = if norm == 0.0 { 0.0 } else { 1.0 / norm };
        for row in 0..3 {
            r[row][col] = m[row][col] * inv;
        }
    }
    isaacsim_core_math::transform::rotation_matrix_to_quaternion(&r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stage::Stage;
    use crate::value::Value;

    #[test]
    fn test_world_transform_composes_ancestors() {
        let mut stage = Stage::new();
        stage.define_prim("/a/b", "Xform").unwrap();
        stage
            .set_attribute("/a", "xformOp:translate", Value::Vec3d([1.0, 0.0, 0.0]))
            .unwrap();
        // 90 deg about Z on /a
        let s = 0.5_f64.sqrt();
        stage
            .set_attribute("/a", "xformOp:orient", Value::Quatd([s, 0.0, 0.0, s]))
            .unwrap();
        stage
            .set_attribute("/a/b", "xformOp:translate", Value::Vec3d([1.0, 0.0, 0.0]))
            .unwrap();

        let world = local_to_world_transform(&stage, "/a/b");
        let t = extract_translation(&world);
        // /a/b's +X offset is rotated to +Y by /a's rotation, then offset
        assert!((t[0] - 1.0).abs() < 1e-12, "{t:?}");
        assert!((t[1] - 1.0).abs() < 1e-12, "{t:?}");
        assert!(t[2].abs() < 1e-12, "{t:?}");

        let q = extract_rotation_quaternion(&world);
        assert!((q[0] - s).abs() < 1e-9);
        assert!((q[3] - s).abs() < 1e-9);
    }

    #[test]
    fn test_scale_does_not_corrupt_rotation() {
        let mut stage = Stage::new();
        stage.define_prim("/a", "Xform").unwrap();
        stage
            .set_attribute("/a", "xformOp:scale", Value::Vec3d([2.0, 3.0, 4.0]))
            .unwrap();
        let q = extract_rotation_quaternion(&local_to_world_transform(&stage, "/a"));
        assert!((q[0] - 1.0).abs() < 1e-12);
    }
}
