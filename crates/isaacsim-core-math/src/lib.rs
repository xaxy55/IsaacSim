// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Math and transform utilities for Isaac Sim.
//!
//! Rust port of the transform subset of the legacy
//! `isaacsim.core.experimental.utils` extension
//! (`legacy/source/extensions/isaacsim.core.experimental.utils/python/impl/transform.py`).
//!
//! Quaternions use `[w, x, y, z]` component order, matrices are 3x3 / 4x4
//! row-indexed arrays (`m[row][col]`), and 4x4 transforms follow the USD
//! row-major convention (translation in the last row) unless stated
//! otherwise. The legacy module dispatches Warp kernels over batches; the
//! Rust port operates on single values — callers iterate for batches.

pub mod linalg;
pub mod transform;

pub use transform::{
    compute_relative_transform, euler_angles_to_quaternion, euler_angles_to_rotation_matrix,
    look_at_matrix, look_at_quaternion, quaternion_conjugate, quaternion_multiplication,
    quaternion_to_euler_angles, quaternion_to_rotation_matrix, rotation_matrix_to_quaternion,
};
