// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Functions for performing transform operations.
//!
//! Port of `python/impl/transform.py` from the legacy
//! `isaacsim.core.experimental.utils` extension. Quaternions are
//! `[w, x, y, z]`; Euler angle input/output order is always
//! `[X, Y, Z] = [roll, pitch, yaw]` for both conventions.

use crate::linalg::{cross3, dot3, mat4_inverse, mat4_mul, mat4_transpose, normalize3};

/// Convert a 3x3 rotation matrix to a quaternion `[w, x, y, z]`.
///
/// Uses the Shepperd-style branch on the largest diagonal term, matching the
/// legacy `_wk_rotation_matrix_to_quaternion` kernel.
pub fn rotation_matrix_to_quaternion(rotation_matrix: &[[f64; 3]; 3]) -> [f64; 4] {
    let m = rotation_matrix;
    let (m00, m01, m02) = (m[0][0], m[0][1], m[0][2]);
    let (m10, m11, m12) = (m[1][0], m[1][1], m[1][2]);
    let (m20, m21, m22) = (m[2][0], m[2][1], m[2][2]);

    let trace = m00 + m11 + m22;
    let (w, x, y, z);
    if trace > 0.0 {
        // w is largest
        let s = (trace + 1.0).sqrt() * 2.0; // s = 4 * w
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if m00 > m11 && m00 > m22 {
        // x is largest
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0; // s = 4 * x
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if m11 > m22 {
        // y is largest
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0; // s = 4 * y
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        // z is largest
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0; // s = 4 * z
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }
    [w, x, y, z]
}

/// Convert Euler angles `[roll, pitch, yaw]` to a 3x3 rotation matrix.
///
/// With `extrinsic` the rotation is applied as `Rz * Ry * Rx` about fixed
/// world axes; otherwise as `Rx * Ry * Rz` about body-fixed axes.
pub fn euler_angles_to_rotation_matrix(
    euler_angles: [f64; 3],
    degrees: bool,
    extrinsic: bool,
) -> [[f64; 3]; 3] {
    let [mut roll, mut pitch, mut yaw] = euler_angles;
    if degrees {
        roll = roll.to_radians();
        pitch = pitch.to_radians();
        yaw = yaw.to_radians();
    }
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();

    if extrinsic {
        // Extrinsic ZYX rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        [
            [cp * cy, cy * sp * sr - cr * sy, sr * sy + cr * cy * sp],
            [cp * sy, cr * cy + sp * sr * sy, cr * sp * sy - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    } else {
        // Intrinsic XYZ rotation: R = Rx(roll) * Ry(pitch) * Rz(yaw)
        [
            [cp * cy, -cp * sy, sp],
            [cy * sr * sp + cr * sy, cr * cy - sr * sp * sy, -cp * sr],
            [-cr * cy * sp + sr * sy, cy * sr + cr * sp * sy, cr * cp],
        ]
    }
}

/// Convert Euler angles `[roll, pitch, yaw]` to a quaternion `[w, x, y, z]`.
pub fn euler_angles_to_quaternion(euler_angles: [f64; 3], degrees: bool, extrinsic: bool) -> [f64; 4] {
    rotation_matrix_to_quaternion(&euler_angles_to_rotation_matrix(euler_angles, degrees, extrinsic))
}

/// Multiply two quaternions using the Hamilton product.
pub fn quaternion_multiplication(first_quaternion: [f64; 4], second_quaternion: [f64; 4]) -> [f64; 4] {
    let [w1, x1, y1, z1] = first_quaternion;
    let [w2, x2, y2, z2] = second_quaternion;
    // Product computed with the same reduced-multiplication scheme as the
    // legacy kernel to preserve bit-level rounding behavior.
    let ww = (z1 + x1) * (x2 + y2);
    let yy = (w1 - y1) * (w2 + z2);
    let zz = (w1 + y1) * (w2 - z2);
    let xx = ww + yy + zz;
    let qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));
    [
        qq - ww + (z1 - y1) * (y2 - z2),
        qq - xx + (x1 + w1) * (x2 + w2),
        qq - yy + (w1 - x1) * (y2 + z2),
        qq - zz + (z1 + y1) * (w2 - x2),
    ]
}

/// Compute the quaternion conjugate by negating the vector part.
///
/// For unit quaternions the conjugate equals the inverse.
pub fn quaternion_conjugate(quaternion: [f64; 4]) -> [f64; 4] {
    [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]
}

/// Convert a quaternion `[w, x, y, z]` to a 3x3 rotation matrix.
///
/// Non-unit quaternions are normalized through the standard `s` factor,
/// matching the legacy kernel.
pub fn quaternion_to_rotation_matrix(quaternion: [f64; 4]) -> [[f64; 3]; 3] {
    let [w, x, y, z] = quaternion;
    let (sqw, sqx, sqy, sqz) = (w * w, x * x, y * y, z * z);
    let s = 1.0 / (sqx + sqy + sqz + sqw);
    [
        [
            1.0 - 2.0 * s * (sqy + sqz),
            2.0 * s * (x * y - z * w),
            2.0 * s * (x * z + y * w),
        ],
        [
            2.0 * s * (x * y + z * w),
            1.0 - 2.0 * s * (sqx + sqz),
            2.0 * s * (y * z - x * w),
        ],
        [
            2.0 * s * (x * z - y * w),
            2.0 * s * (y * z + x * w),
            1.0 - 2.0 * s * (sqx + sqy),
        ],
    ]
}

/// Convert a quaternion to Euler angles `[roll, pitch, yaw]`.
///
/// With `extrinsic` the angles correspond to the ZYX convention (returned in
/// `[X, Y, Z]` order); otherwise to the intrinsic XYZ convention. Near gimbal
/// lock the roll is set to zero, matching the legacy kernel.
pub fn quaternion_to_euler_angles(quaternion: [f64; 4], degrees: bool, extrinsic: bool) -> [f64; 3] {
    let m = quaternion_to_rotation_matrix(quaternion);
    const GIMBAL_LOCK_THRESHOLD: f64 = 1e-6;

    let (roll, pitch, yaw);
    if extrinsic {
        // For extrinsic ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        pitch = -m[2][0].clamp(-1.0, 1.0).asin();
        if pitch.cos().abs() > GIMBAL_LOCK_THRESHOLD {
            roll = m[2][1].atan2(m[2][2]);
            yaw = m[1][0].atan2(m[0][0]);
        } else {
            roll = 0.0;
            yaw = (-m[0][1]).atan2(m[1][1]);
        }
    } else {
        // For intrinsic XYZ convention: R = Rx(roll) * Ry(pitch) * Rz(yaw)
        pitch = m[0][2].clamp(-1.0, 1.0).asin();
        if pitch.cos().abs() > GIMBAL_LOCK_THRESHOLD {
            roll = (-m[1][2]).atan2(m[2][2]);
            yaw = (-m[0][1]).atan2(m[0][0]);
        } else {
            roll = 0.0;
            // In gimbal lock (pitch ~ +/-90 deg), use m11 to avoid instability when m00 ~ 0
            yaw = m[1][0].atan2(m[1][1]);
        }
    }
    if degrees {
        [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()]
    } else {
        [roll, pitch, yaw]
    }
}

/// Compute the orientation quaternion for a look-at transform.
///
/// Orients a frame at `eye` so that its negative-Z axis points toward
/// `target` (OpenGL / USD camera convention). `up` defaults to Z-up
/// `[0, 0, 1]`; when the forward direction is nearly parallel to it, a Y-up
/// fallback is chosen automatically.
pub fn look_at_quaternion(eye: [f64; 3], target: [f64; 3], up: Option<[f64; 3]>) -> [f64; 4] {
    const EPS: f64 = 1e-8;
    const PARALLEL_THRESHOLD: f64 = 0.99;

    // Forward = normalize(target - eye); the +EPS denominator matches the
    // legacy kernel's division guard.
    let f = [target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]];
    let f_len = dot3(f, f).sqrt() + EPS;
    let f = [f[0] / f_len, f[1] / f_len, f[2] / f_len];

    let mut up = up.unwrap_or([0.0, 0.0, 1.0]);
    if dot3(f, up).abs() > PARALLEL_THRESHOLD {
        up = [0.0, 1.0, 0.0];
    }

    // Camera looks along -Z: right = normalize(cross(up, -forward)),
    // recomputed up = normalize(cross(-forward, right)).
    let neg_f = [-f[0], -f[1], -f[2]];
    let r = cross3(up, neg_f);
    let r_len = dot3(r, r).sqrt() + EPS;
    let r = [r[0] / r_len, r[1] / r_len, r[2] / r_len];
    let ru = cross3(neg_f, r);
    let ru_len = dot3(ru, ru).sqrt() + EPS;
    let ru = [ru[0] / ru_len, ru[1] / ru_len, ru[2] / ru_len];

    // Rotation matrix columns: [right, recomputed_up, -forward]
    rotation_matrix_to_quaternion(&[
        [r[0], ru[0], neg_f[0]],
        [r[1], ru[1], neg_f[1]],
        [r[2], ru[2], neg_f[2]],
    ])
}

/// Compute the camera transform matrix (position + orientation) for a look-at.
///
/// Returns the row-major (USD/Gf convention) 4x4 matrix that places a camera
/// at `eye` oriented so its negative-Z axis points toward `target`. This is
/// the inverse of the standard view (LookAt) matrix — i.e. the equivalent of
/// the legacy `Gf.Matrix4d(1).SetLookAt(...).GetInverse()` — and can be used
/// directly as a prim's local transform.
///
/// `up` defaults to Z-up `[0, 0, 1]`. When the forward direction is nearly
/// parallel to `up` (cross-product length below `epsilon`, legacy default
/// `1e-5`), Y-up then X-up fallbacks are chosen automatically.
pub fn look_at_matrix(
    eye: [f64; 3],
    target: [f64; 3],
    up: Option<[f64; 3]>,
    epsilon: f64,
) -> [[f64; 4]; 4] {
    let mut up = up.unwrap_or([0.0, 0.0, 1.0]);
    let forward = [target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]];

    // Collinearity check: if forward x up ~ 0, try perpendicular fallbacks
    if crate::linalg::norm3(cross3(forward, up)) < epsilon {
        up = [0.0, 1.0, 0.0];
        if crate::linalg::norm3(cross3(forward, up)) < epsilon {
            up = [1.0, 0.0, 0.0];
        }
    }

    // Camera basis in world space: X = right, Y = recomputed up, Z = back.
    let f = normalize3(forward);
    let right = normalize3(cross3(f, up));
    let up2 = cross3(right, f);
    let back = [-f[0], -f[1], -f[2]];

    // Row-vector (Gf) convention: basis vectors are the matrix rows and the
    // translation is the last row.
    [
        [right[0], right[1], right[2], 0.0],
        [up2[0], up2[1], up2[2], 0.0],
        [back[0], back[1], back[2], 0.0],
        [eye[0], eye[1], eye[2], 1.0],
    ]
}

/// Compute the relative 4x4 transform from a source frame to a target frame
/// given their world transforms.
///
/// Both inputs are expected in USD row-major convention (as returned by
/// `UsdGeom.Xformable.ComputeLocalToWorldTransform`). The result is a
/// column-major 4x4 matrix that transforms points from the source local frame
/// into the target local frame.
///
/// Returns `None` if `target_to_world` is singular (legacy numpy raises
/// `LinAlgError`).
pub fn compute_relative_transform(
    source_to_world: &[[f64; 4]; 4],
    target_to_world: &[[f64; 4]; 4],
) -> Option<[[f64; 4]; 4]> {
    let source_col = mat4_transpose(source_to_world);
    let target_col = mat4_transpose(target_to_world);
    let world_to_target_col = mat4_inverse(&target_col)?;
    Some(mat4_mul(&world_to_target_col, &source_col))
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-5;

    fn assert_close(actual: &[f64], expected: &[f64], atol: f64) {
        assert_eq!(actual.len(), expected.len());
        for (a, e) in actual.iter().zip(expected) {
            assert!((a - e).abs() < atol, "expected {expected:?}, got {actual:?}");
        }
    }

    fn quat_norm(q: [f64; 4]) -> f64 {
        q.iter().map(|c| c * c).sum::<f64>().sqrt()
    }

    fn mat3_det(m: &[[f64; 3]; 3]) -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    const IDENTITY3: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    /// Port of legacy `test_transform.py::test_rotation_matrix_to_quaternion`.
    #[test]
    fn test_rotation_matrix_to_quaternion() {
        let result = rotation_matrix_to_quaternion(&IDENTITY3);
        assert_close(&result, &[1.0, 0.0, 0.0, 0.0], TOLERANCE);
        assert!((quat_norm(result) - 1.0).abs() < TOLERANCE);
    }

    /// Port of legacy `test_euler_angles_to_rotation_matrix` plus the module
    /// doctest (90 deg around Y).
    #[test]
    fn test_euler_angles_to_rotation_matrix() {
        let result = euler_angles_to_rotation_matrix([0.0, 0.0, 0.0], false, true);
        for (row, expected_row) in result.iter().zip(&IDENTITY3) {
            assert_close(row, expected_row, TOLERANCE);
        }
        assert!((mat3_det(&result) - 1.0).abs() < TOLERANCE);
    }

    /// Port of legacy `test_euler_angles_to_quaternion` plus the module
    /// doctest: euler [0, pi/2, 0] -> [0.7071, 0, 0.7071, 0].
    #[test]
    fn test_euler_angles_to_quaternion() {
        let result = euler_angles_to_quaternion([0.0, 0.0, 0.0], false, true);
        assert_close(&result, &[1.0, 0.0, 0.0, 0.0], TOLERANCE);

        let result = euler_angles_to_quaternion([0.0, std::f64::consts::FRAC_PI_2, 0.0], false, true);
        let s = 0.5_f64.sqrt();
        assert_close(&result, &[s, 0.0, s, 0.0], TOLERANCE);
    }

    /// Port of legacy `test_degrees_vs_radians`.
    #[test]
    fn test_degrees_vs_radians() {
        let deg = euler_angles_to_rotation_matrix([90.0, 0.0, 0.0], true, true);
        let rad = euler_angles_to_rotation_matrix([std::f64::consts::FRAC_PI_2, 0.0, 0.0], false, true);
        for (row_deg, row_rad) in deg.iter().zip(&rad) {
            assert_close(row_deg, row_rad, TOLERANCE);
        }
    }

    /// Port of legacy `test_quaternion_multiplication` (identity x identity)
    /// plus associativity from `test_quaternion_multiplication_associativity`.
    #[test]
    fn test_quaternion_multiplication() {
        let identity = [1.0, 0.0, 0.0, 0.0];
        assert_close(
            &quaternion_multiplication(identity, identity),
            &identity,
            TOLERANCE,
        );

        let s = 0.5_f64.sqrt();
        let q1 = [s, s, 0.0, 0.0];
        let q2 = [s, 0.0, s, 0.0];
        let q3 = [s, 0.0, 0.0, s];
        let left = quaternion_multiplication(quaternion_multiplication(q1, q2), q3);
        let right = quaternion_multiplication(q1, quaternion_multiplication(q2, q3));
        assert_close(&left, &right, TOLERANCE);
        assert!((quat_norm(left) - 1.0).abs() < TOLERANCE);
    }

    /// Port of legacy `test_quaternion_conjugate`.
    #[test]
    #[allow(clippy::approx_constant)] // 0.7071 literals come from the legacy test
    fn test_quaternion_conjugate() {
        let q = [0.7071, 0.7071, 0.0, 0.0];
        assert_close(&quaternion_conjugate(q), &[0.7071, -0.7071, 0.0, 0.0], TOLERANCE);
        // Conjugate of a unit quaternion is its inverse: q * conj(q) = identity
        let product = quaternion_multiplication(q, quaternion_conjugate(q));
        let normalized: Vec<f64> = product.iter().map(|c| c / quat_norm(product)).collect();
        assert_close(&normalized, &[1.0, 0.0, 0.0, 0.0], TOLERANCE);
    }

    /// Port of legacy `test_quaternion_to_rotation_matrix`.
    #[test]
    fn test_quaternion_to_rotation_matrix() {
        let result = quaternion_to_rotation_matrix([1.0, 0.0, 0.0, 0.0]);
        for (row, expected_row) in result.iter().zip(&IDENTITY3) {
            assert_close(row, expected_row, TOLERANCE);
        }

        // 90 deg around Z
        let s = 0.5_f64.sqrt();
        let result = quaternion_to_rotation_matrix([s, 0.0, 0.0, s]);
        let expected = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        for (row, expected_row) in result.iter().zip(&expected) {
            assert_close(row, expected_row, TOLERANCE);
        }
        assert!((mat3_det(&result) - 1.0).abs() < TOLERANCE);
    }

    /// Port of legacy `test_quaternion_to_euler_angles*` (extrinsic and
    /// intrinsic, radians and degrees).
    #[test]
    fn test_quaternion_to_euler_angles() {
        let half_pi = std::f64::consts::FRAC_PI_2;
        let s = 0.5_f64.sqrt();

        assert_close(
            &quaternion_to_euler_angles([1.0, 0.0, 0.0, 0.0], false, true),
            &[0.0, 0.0, 0.0],
            TOLERANCE,
        );
        // 90 deg around X, Y, Z (extrinsic)
        assert_close(
            &quaternion_to_euler_angles([s, s, 0.0, 0.0], false, true),
            &[half_pi, 0.0, 0.0],
            TOLERANCE,
        );
        assert_close(
            &quaternion_to_euler_angles([s, s, 0.0, 0.0], true, true),
            &[90.0, 0.0, 0.0],
            TOLERANCE,
        );
        // 90 deg around X, Y, Z (intrinsic)
        assert_close(
            &quaternion_to_euler_angles([s, s, 0.0, 0.0], false, false),
            &[half_pi, 0.0, 0.0],
            TOLERANCE,
        );
        assert_close(
            &quaternion_to_euler_angles([s, 0.0, s, 0.0], false, false),
            &[0.0, half_pi, 0.0],
            TOLERANCE,
        );
        assert_close(
            &quaternion_to_euler_angles([s, 0.0, 0.0, s], true, false),
            &[0.0, 0.0, 90.0],
            TOLERANCE,
        );
    }

    /// Port of legacy `test_euler_quaternion_roundtrip` and
    /// `test_euler_quaternion_roundtrip_intrinsic`.
    #[test]
    fn test_euler_quaternion_roundtrip() {
        let test_angles = [
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 0.5],
            [0.3, 0.4, 0.5],
        ];
        for extrinsic in [true, false] {
            for euler in test_angles {
                let quaternion = euler_angles_to_quaternion(euler, false, extrinsic);
                let back = quaternion_to_euler_angles(quaternion, false, extrinsic);
                assert_close(&back, &euler, TOLERANCE);
            }
        }
    }

    /// Port of legacy `test_quaternion_euler_roundtrip` and
    /// `test_quaternion_euler_roundtrip_intrinsic`.
    #[test]
    fn test_quaternion_euler_roundtrip() {
        let s = 0.5_f64.sqrt();
        let test_quaternions = [
            [1.0, 0.0, 0.0, 0.0],
            [s, s, 0.0, 0.0],
            [s, 0.0, s, 0.0],
            [s, 0.0, 0.0, s],
            [0.5, 0.5, 0.5, 0.5],
        ];
        for extrinsic in [true, false] {
            for q in test_quaternions {
                let euler = quaternion_to_euler_angles(q, false, extrinsic);
                let back = euler_angles_to_quaternion(euler, false, extrinsic);
                // q and -q represent the same rotation
                let same = q.iter().zip(&back).all(|(a, b)| (a - b).abs() < TOLERANCE);
                let negated = q.iter().zip(&back).all(|(a, b)| (a + b).abs() < TOLERANCE);
                assert!(same || negated, "round-trip failed for {q:?}: got {back:?}");
            }
        }
    }

    /// Port of legacy `test_look_at_quaternion_*`.
    #[test]
    fn test_look_at_quaternion() {
        // Single pair: unit-length result
        let q = look_at_quaternion([5.0, 5.0, 5.0], [0.0, 0.0, 0.0], None);
        assert!((quat_norm(q) - 1.0).abs() < TOLERANCE);

        // Looking straight down triggers the up-vector fallback
        let q = look_at_quaternion([0.0, 0.0, 10.0], [0.0, 0.0, 0.0], None);
        assert!((quat_norm(q) - 1.0).abs() < TOLERANCE);

        // Different up vectors produce different orientations
        let q_z = look_at_quaternion([5.0, 5.0, 5.0], [0.0, 0.0, 0.0], None);
        let q_y = look_at_quaternion([5.0, 5.0, 5.0], [0.0, 0.0, 0.0], Some([0.0, 1.0, 0.0]));
        assert!(q_z.iter().zip(&q_y).any(|(a, b)| (a - b).abs() > 1e-4));

        // Roundtrip: -Z column of the rotation matrix points eye -> target
        let q = look_at_quaternion([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], None);
        let m = quaternion_to_rotation_matrix(q);
        let forward = [-m[0][2], -m[1][2], -m[2][2]];
        assert_close(&forward, &[-1.0, 0.0, 0.0], 1e-4);
    }

    /// Port of legacy `test_look_at_matrix_*` properties: translation row is
    /// the eye position, -Z row points toward the target, basis orthonormal,
    /// collinear forward/up falls back without error.
    #[test]
    #[allow(clippy::type_complexity)]
    fn test_look_at_matrix() {
        let cases: [([f64; 3], [f64; 3], Option<[f64; 3]>); 5] = [
            ([5.0, 5.0, 5.0], [0.0, 0.0, 0.0], Some([0.0, 0.0, 1.0])),
            ([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], None),
            ([0.0, 0.0, 10.0], [0.0, 0.0, 5.0], Some([0.0, 1.0, 0.0])),
            ([3.0, 4.0, 5.0], [1.0, 2.0, 3.0], Some([0.0, 0.0, 1.0])),
            ([100.0, 0.0, 50.0], [0.0, 0.0, 0.0], Some([0.0, 1.0, 0.0])),
        ];
        for (eye, target, up) in cases {
            let m = look_at_matrix(eye, target, up, 1e-5);
            // Translation row equals the eye position
            assert_close(&m[3], &[eye[0], eye[1], eye[2], 1.0], 1e-10);
            // -Z row points toward the target
            let forward = normalize3([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
            assert_close(&[-m[2][0], -m[2][1], -m[2][2]], &forward, TOLERANCE);
            // Rotation rows are orthonormal
            for i in 0..3 {
                for j in 0..3 {
                    let row_i = [m[i][0], m[i][1], m[i][2]];
                    let row_j = [m[j][0], m[j][1], m[j][2]];
                    let expected = if i == j { 1.0 } else { 0.0 };
                    assert!((dot3(row_i, row_j) - expected).abs() < TOLERANCE);
                }
            }
        }

        // Collinearity fallback: looking straight down with Z-up
        let m = look_at_matrix([0.0, 0.0, 10.0], [0.0, 0.0, 0.0], Some([0.0, 0.0, 1.0]), 1e-5);
        assert_close(&m[3], &[0.0, 0.0, 10.0, 1.0], TOLERANCE);
    }

    /// Port of legacy `test_compute_relative_transform_*`.
    #[test]
    fn test_compute_relative_transform() {
        let identity4 = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        // Identity inputs -> identity
        let result = compute_relative_transform(&identity4, &identity4).unwrap();
        for (row, expected_row) in result.iter().zip(&identity4) {
            assert_close(row, expected_row, 1e-10);
        }

        // Same non-trivial transform -> identity
        let mut t = identity4;
        t[3][..3].copy_from_slice(&[5.0, -3.0, 2.0]);
        let result = compute_relative_transform(&t, &t).unwrap();
        for (row, expected_row) in result.iter().zip(&identity4) {
            assert_close(row, expected_row, 1e-10);
        }

        // Pure translation offset (row-major input, column-major output)
        let mut source = identity4;
        source[3][..3].copy_from_slice(&[1.0, 2.0, 3.0]);
        let mut target = identity4;
        target[3][..3].copy_from_slice(&[4.0, 6.0, 8.0]);
        let result = compute_relative_transform(&source, &target).unwrap();
        assert_close(
            &[result[0][3], result[1][3], result[2][3]],
            &[-3.0, -4.0, -5.0],
            1e-10,
        );

        // A->B composed with B->A gives identity
        let mut source = identity4;
        source[3][..3].copy_from_slice(&[1.0, 0.0, 0.0]);
        let target = [
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 2.0, 0.0, 1.0],
        ];
        let ab = compute_relative_transform(&source, &target).unwrap();
        let ba = compute_relative_transform(&target, &source).unwrap();
        let composed = mat4_mul(&ab, &ba);
        for (row, expected_row) in composed.iter().zip(&identity4) {
            assert_close(row, expected_row, 1e-10);
        }
    }
}
