// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Typed attribute values (the subset of `Sdf.ValueTypeNames` the ported
//! code needs).

/// A typed attribute value. Quaternions are `[w, x, y, z]`.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Float(f32),
    Double(f64),
    String(String),
    Token(String),
    Vec3f([f32; 3]),
    Vec3d([f64; 3]),
    Quatf([f32; 4]),
    Quatd([f64; 4]),
    TokenArray(Vec<String>),
}

impl Value {
    /// The value as a double-precision 3-vector, widening `Vec3f` if needed.
    pub fn as_vec3d(&self) -> Option<[f64; 3]> {
        match self {
            Value::Vec3d(v) => Some(*v),
            Value::Vec3f(v) => Some([v[0] as f64, v[1] as f64, v[2] as f64]),
            _ => None,
        }
    }

    /// The value as a double-precision quaternion `[w, x, y, z]`, widening
    /// `Quatf` if needed.
    pub fn as_quatd(&self) -> Option<[f64; 4]> {
        match self {
            Value::Quatd(q) => Some(*q),
            Value::Quatf(q) => Some([q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64]),
            _ => None,
        }
    }
}
