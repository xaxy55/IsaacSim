// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Small fixed-size linear-algebra helpers backing the transform utilities.
//!
//! The legacy Python code leans on numpy (`np.linalg.solve`, `np.linalg.inv`,
//! matrix products); these are the minimal equivalents for 3- and 4-dimensional
//! problems.

/// Dot product of two 3-vectors.
pub fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product of two 3-vectors.
pub fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean norm of a 3-vector.
pub fn norm3(v: [f64; 3]) -> f64 {
    dot3(v, v).sqrt()
}

/// Normalize a 3-vector. Returns the zero vector unchanged.
pub fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let n = norm3(v);
    if n == 0.0 {
        v
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

/// Multiply a 3x3 matrix by a column 3-vector (`M * v`).
pub fn mat3_mul_vec3(m: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Solve the 3x3 linear system `A x = b` by Gaussian elimination with
/// partial pivoting. Returns `None` if `A` is singular.
pub fn solve3(a: &[[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    // Augmented matrix [A | b]
    let mut m = [
        [a[0][0], a[0][1], a[0][2], b[0]],
        [a[1][0], a[1][1], a[1][2], b[1]],
        [a[2][0], a[2][1], a[2][2], b[2]],
    ];
    for col in 0..3 {
        let pivot_row = (col..3).max_by(|&i, &j| {
            m[i][col]
                .abs()
                .partial_cmp(&m[j][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if m[pivot_row][col] == 0.0 {
            return None;
        }
        m.swap(col, pivot_row);
        let pivot_values = m[col];
        for (row, row_values) in m.iter_mut().enumerate() {
            if row != col {
                let factor = row_values[col] / pivot_values[col];
                for (k, pivot_value) in pivot_values.iter().enumerate().skip(col) {
                    row_values[k] -= factor * pivot_value;
                }
            }
        }
    }
    Some([m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]])
}

/// Multiply two 4x4 matrices (`A * B`).
pub fn mat4_mul(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for (i, row) in a.iter().enumerate() {
        for j in 0..4 {
            out[i][j] = (0..4).map(|k| row[k] * b[k][j]).sum();
        }
    }
    out
}

/// Transpose of a 4x4 matrix.
pub fn mat4_transpose(m: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for (i, row) in m.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            out[j][i] = *value;
        }
    }
    out
}

/// Invert a 4x4 matrix by Gauss-Jordan elimination with partial pivoting.
/// Returns `None` if the matrix is singular.
pub fn mat4_inverse(m: &[[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut a = *m;
    let mut inv = [[0.0; 4]; 4];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..4 {
        let pivot_row = (col..4).max_by(|&i, &j| {
            a[i][col]
                .abs()
                .partial_cmp(&a[j][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if a[pivot_row][col] == 0.0 {
            return None;
        }
        a.swap(col, pivot_row);
        inv.swap(col, pivot_row);
        let pivot = a[col][col];
        for k in 0..4 {
            a[col][k] /= pivot;
            inv[col][k] /= pivot;
        }
        for row in 0..4 {
            if row != col {
                let factor = a[row][col];
                for k in 0..4 {
                    a[row][k] -= factor * a[col][k];
                    inv[row][k] -= factor * inv[col][k];
                }
            }
        }
    }
    Some(inv)
}

/// Eigendecomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations.
///
/// Returns `(eigenvalues, eigenvectors)` where `eigenvectors[i]` is the unit
/// eigenvector for `eigenvalues[i]`. Used to express inertia tensors as
/// diagonal inertia plus principal axes.
pub fn sym3_eigen(m: &[[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = *m;
    // v accumulates the rotations; columns are eigenvectors
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        // Largest off-diagonal element
        let off = [(0, 1), (0, 2), (1, 2)];
        let &(p, q) = off
            .iter()
            .max_by(|&&(i, j), &&(k, l)| {
                a[i][j]
                    .abs()
                    .partial_cmp(&a[k][l].abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .expect("non-empty");
        if a[p][q].abs() < 1e-15 {
            break;
        }
        // Jacobi rotation annihilating a[p][q]
        let theta = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let (s, c) = theta.sin_cos();
        let mut r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        r[p][p] = c;
        r[q][q] = c;
        r[p][q] = s;
        r[q][p] = -s;
        // a = r^T a r ; v = v r
        let rt_a = mat3_mul_generic(&mat3_transpose_generic(&r), &a);
        a = mat3_mul_generic(&rt_a, &r);
        v = mat3_mul_generic(&v, &r);
    }
    let eigenvalues = [a[0][0], a[1][1], a[2][2]];
    let eigenvectors = [
        [v[0][0], v[1][0], v[2][0]],
        [v[0][1], v[1][1], v[2][1]],
        [v[0][2], v[1][2], v[2][2]],
    ];
    (eigenvalues, eigenvectors)
}

fn mat3_mul_generic(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in a.iter().enumerate() {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| row[k] * b[k][j]).sum();
        }
    }
    out
}

fn mat3_transpose_generic(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
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
    fn test_solve3() {
        let a = [[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [1.0, 0.0, 1.0]];
        let x = solve3(&a, [4.0, 9.0, 5.0]).unwrap();
        assert_eq!(x, [2.0, 3.0, 3.0]);
        let singular = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]];
        assert!(solve3(&singular, [1.0, 2.0, 3.0]).is_none());
    }

    #[test]
    fn test_sym3_eigen() {
        // Diagonal matrix: eigenvalues are the diagonal, axes are identity
        let (values, vectors) = sym3_eigen(&[[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 5.0]]);
        assert_eq!(values, [2.0, 3.0, 5.0]);
        for (i, vector) in vectors.iter().enumerate() {
            for (j, component) in vector.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((component - expected).abs() < 1e-12);
            }
        }

        // Symmetric matrix with known decomposition:
        // [[2,1,0],[1,2,0],[0,0,3]] has eigenvalues 1, 3, 3
        let m = [[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 3.0]];
        let (values, vectors) = sym3_eigen(&m);
        let mut sorted = values;
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 1.0).abs() < 1e-12);
        assert!((sorted[1] - 3.0).abs() < 1e-12);
        assert!((sorted[2] - 3.0).abs() < 1e-12);
        // Each eigenpair satisfies M v = lambda v
        for (lambda, v) in values.iter().zip(&vectors) {
            let mv = mat3_mul_vec3(&m, *v);
            for (a, b) in mv.iter().zip(v) {
                assert!((a - lambda * b).abs() < 1e-12, "Mv != lambda v");
            }
        }
    }

    #[test]
    fn test_mat4_inverse_roundtrip() {
        let m = [
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [3.0, 2.0, 1.0, 1.0],
        ];
        let inv = mat4_inverse(&m).unwrap();
        let identity = mat4_mul(&m, &inv);
        for (i, row) in identity.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((value - expected).abs() < 1e-12);
            }
        }
        let singular = [[0.0; 4]; 4];
        assert!(mat4_inverse(&singular).is_none());
    }
}
