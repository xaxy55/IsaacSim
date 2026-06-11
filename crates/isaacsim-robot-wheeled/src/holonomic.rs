// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Holonomic (e.g. mecanum) controller for wheeled robots.
//!
//! Port of `controllers/holonomic_controller.py`. The legacy implementation
//! solves a quadratic program with OSQP:
//!
//! ```text
//! minimize    x' P x          (P = diag(wheel_radius / |wheel_radius|))
//! subject to  V_x x = v_x,  V_y x = v_y,  W_z x = w
//! ```
//!
//! where the inequality rows are unbounded, so the QP is an
//! equality-constrained least-norm problem with the closed-form KKT solution
//! `x = P^-1 E' (E P^-1 E')^-1 b`. This port solves that system directly
//! instead of binding a QP solver; results match the legacy OSQP solution to
//! solver tolerance.

use isaacsim_core_math::linalg::{mat3_mul_vec3, normalize3, solve3};
use isaacsim_core_math::transform::{euler_angles_to_quaternion, quaternion_to_rotation_matrix};

/// Configuration for [`HolonomicController`].
///
/// `wheel_radius` and `mecanum_angles` accept either one entry per wheel or a
/// single entry broadcast to all wheels, matching the legacy scalar handling.
#[derive(Debug, Clone)]
pub struct HolonomicConfig {
    /// Radius of each wheel in m.
    pub wheel_radius: Vec<f64>,
    /// Positions of each wheel relative to the robot center.
    pub wheel_positions: Vec<[f64; 3]>,
    /// Quaternion orientations of each wheel as `[w, x, y, z]`.
    pub wheel_orientations: Vec<[f64; 4]>,
    /// Mecanum roller angles in degrees for each wheel.
    pub mecanum_angles: Vec<f64>,
    /// Local rotation axis of the wheel joint (legacy default `[1, 0, 0]`).
    pub wheel_axis: [f64; 3],
    /// Up direction of the robot frame (legacy default `[0, 0, 1]`).
    pub up_axis: [f64; 3],
    /// Maximum linear speed in m/s.
    pub max_linear_speed: f64,
    /// Maximum angular speed in rad/s.
    pub max_angular_speed: f64,
    /// Maximum individual wheel speed in rad/s.
    pub max_wheel_speed: f64,
    /// Gain applied to the linear velocity command.
    pub linear_gain: f64,
    /// Gain applied to the angular velocity command.
    pub angular_gain: f64,
}

impl Default for HolonomicConfig {
    fn default() -> Self {
        Self {
            wheel_radius: Vec::new(),
            wheel_positions: Vec::new(),
            wheel_orientations: Vec::new(),
            mecanum_angles: Vec::new(),
            wheel_axis: [1.0, 0.0, 0.0],
            up_axis: [0.0, 0.0, 1.0],
            max_linear_speed: f64::INFINITY,
            max_angular_speed: f64::INFINITY,
            max_wheel_speed: f64::INFINITY,
            linear_gain: 1.0,
            angular_gain: 1.0,
        }
    }
}

/// QP-based holonomic controller for mecanum-wheeled robots.
///
/// Converts `[forward, lateral, yaw]` velocity commands into per-wheel
/// angular velocities.
#[derive(Debug, Clone)]
pub struct HolonomicController {
    /// Number of wheels.
    pub num_wheels: usize,
    /// Wheel drive directions scaled by wheel radius, one `[x, y, z]` column
    /// per wheel (legacy `base_dir`; the z component is always zero).
    pub base_dir: Vec<[f64; 3]>,
    /// Inverse of the diagonal objective weights `P`.
    p_inv: Vec<f64>,
    /// Equality-constraint matrix rows `[V_x; V_y; W_z]`, one column per wheel.
    constraints: Vec<[f64; 3]>,
    /// Maximum linear speed in m/s.
    pub max_linear_speed: f64,
    /// Maximum angular speed in rad/s.
    pub max_angular_speed: f64,
    /// Maximum individual wheel speed in rad/s.
    pub max_wheel_speed: f64,
    /// Gain applied to the linear velocity command.
    pub linear_gain: f64,
    /// Gain applied to the angular velocity command.
    pub angular_gain: f64,
    /// Last computed wheel commands (returned again if a solve fails).
    pub joint_commands: Vec<f64>,
}

impl HolonomicController {
    /// Build a controller from `config`.
    ///
    /// Returns an error if a required array is empty or the per-wheel array
    /// lengths are inconsistent (legacy raises `ValueError` for missing
    /// parameters).
    pub fn new(config: HolonomicConfig) -> Result<Self, String> {
        for (name, len) in [
            ("wheel_radius", config.wheel_radius.len()),
            ("wheel_positions", config.wheel_positions.len()),
            ("wheel_orientations", config.wheel_orientations.len()),
            ("mecanum_angles", config.mecanum_angles.len()),
        ] {
            if len == 0 {
                return Err(format!(
                    "{name} is required (received an empty array). \
                     Pass an array with one entry per wheel."
                ));
            }
        }
        let num_wheels = config.wheel_positions.len();
        let broadcast = |values: &[f64], name: &str| -> Result<Vec<f64>, String> {
            if values.len() == 1 {
                Ok(vec![values[0]; num_wheels])
            } else if values.len() == num_wheels {
                Ok(values.to_vec())
            } else {
                Err(format!(
                    "{name} must have 1 or {num_wheels} entries, got {}",
                    values.len()
                ))
            }
        };
        let wheel_radius = broadcast(&config.wheel_radius, "wheel_radius")?;
        let mecanum_angles = broadcast(&config.mecanum_angles, "mecanum_angles")?;
        if config.wheel_orientations.len() != num_wheels {
            return Err(format!(
                "wheel_orientations must have {num_wheels} entries, got {}",
                config.wheel_orientations.len()
            ));
        }

        // Legacy _build_base: drive direction of wheel i is the wheel joint
        // axis rotated by the wheel orientation, then by the mecanum roller
        // angle about the robot up axis, scaled by the wheel radius.
        let mut base_dir = Vec::with_capacity(num_wheels);
        let mut constraints = Vec::with_capacity(num_wheels);
        for i in 0..num_wheels {
            let r0 = quaternion_to_rotation_matrix(config.wheel_orientations[i]);
            let euler = [
                config.up_axis[0] * mecanum_angles[i],
                config.up_axis[1] * mecanum_angles[i],
                config.up_axis[2] * mecanum_angles[i],
            ];
            let mecanum_quat = euler_angles_to_quaternion(euler, true, true);
            let mecanum_rot = quaternion_to_rotation_matrix(mecanum_quat);
            let j_axis = normalize3(mat3_mul_vec3(
                &mecanum_rot,
                mat3_mul_vec3(&r0, config.wheel_axis),
            ));
            let v = [
                j_axis[0] * wheel_radius[i],
                j_axis[1] * wheel_radius[i],
                0.0,
            ];
            base_dir.push(v);
            // W_z = (V x d)_z with d = (p_x, p_y, 0)
            let p = config.wheel_positions[i];
            constraints.push([v[0], v[1], v[0] * p[1] - v[1] * p[0]]);
        }

        // P = diag(wheel_radius / |wheel_radius|); fall back to identity for
        // an all-zero radius vector, matching the legacy guard.
        let norm_r = wheel_radius.iter().map(|r| r * r).sum::<f64>().sqrt();
        let p_inv: Vec<f64> = if norm_r > 0.0 {
            wheel_radius.iter().map(|r| norm_r / r).collect()
        } else {
            vec![1.0; num_wheels]
        };

        Ok(Self {
            num_wheels,
            base_dir,
            p_inv,
            constraints,
            max_linear_speed: config.max_linear_speed,
            max_angular_speed: config.max_angular_speed,
            max_wheel_speed: config.max_wheel_speed,
            linear_gain: config.linear_gain,
            angular_gain: config.angular_gain,
            joint_commands: vec![0.0; num_wheels],
        })
    }

    /// Compute wheel velocities from a `[forward, lateral, yaw]` command.
    ///
    /// Returns one wheel joint velocity per wheel in rad/s. If the constraint
    /// system is singular the previous commands are returned, matching the
    /// legacy behavior of keeping `joint_commands` on solver failure.
    pub fn forward(&mut self, command: [f64; 3]) -> Vec<f64> {
        if command.iter().all(|c| c.abs() <= 1e-8) {
            return vec![0.0; self.num_wheels];
        }
        let mut v = [
            command[0] * self.linear_gain,
            command[1] * self.linear_gain,
            0.0,
        ];
        let mut w = command[2] * self.angular_gain;
        let v_norm = (v[0] * v[0] + v[1] * v[1]).sqrt();
        if v_norm > self.max_linear_speed {
            let scale = self.max_linear_speed / v_norm;
            v = [v[0] * scale, v[1] * scale, 0.0];
        }
        if w.abs() > self.max_angular_speed {
            w = w.signum() * self.max_angular_speed;
        }

        // Closed-form solution of: minimize x' P x  s.t.  E x = b
        //   x = P^-1 E' y  with  (E P^-1 E') y = b
        let b = [v[0], v[1], w];
        let mut gram = [[0.0; 3]; 3];
        for (col, p_inv) in self.constraints.iter().zip(&self.p_inv) {
            for r in 0..3 {
                for c in 0..3 {
                    gram[r][c] += col[r] * p_inv * col[c];
                }
            }
        }
        let Some(y) = solve3(&gram, b) else {
            return self.joint_commands.clone();
        };
        let mut values: Vec<f64> = self
            .constraints
            .iter()
            .zip(&self.p_inv)
            .map(|(col, p_inv)| p_inv * (col[0] * y[0] + col[1] * y[1] + col[2] * y[2]))
            .collect();

        let max_value = values.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if max_value > self.max_wheel_speed {
            let scale = self.max_wheel_speed / max_value;
            for value in &mut values {
                *value *= scale;
            }
        }
        self.joint_commands = values;
        self.joint_commands.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> HolonomicConfig {
        // Same 3-wheel kiwi-drive configuration as the legacy
        // tests/test_holonomic_controller.py fixtures.
        HolonomicConfig {
            wheel_radius: vec![0.04, 0.04, 0.04],
            wheel_positions: vec![
                [-0.0980432, 0.000636773, -0.050501],
                [0.0493475, -0.084525, -0.050501],
                [0.0495291, 0.0856937, -0.050501],
            ],
            wheel_orientations: vec![
                [0.0, 0.0, 0.0, 1.0],
                [0.866, 0.0, 0.0, -0.5],
                [0.866, 0.0, 0.0, 0.5],
            ],
            mecanum_angles: vec![90.0, 90.0, 90.0],
            ..Default::default()
        }
    }

    /// Port of legacy `tests/test_holonomic_controller.py::test_holonomic_drive`.
    #[test]
    fn test_holonomic_drive() {
        let mut controller = HolonomicController::new(test_config()).unwrap();
        let actions = controller.forward([1.0, 1.0, 0.1]);
        assert_eq!(actions.len(), controller.num_wheels);
        assert!(actions.iter().all(|a| a.is_finite()));
        assert!((actions[0] - -25.105).abs() < 0.01, "got {actions:?}");
        assert!((actions[1] - 14.3182).abs() < 0.01, "got {actions:?}");
        assert!((actions[2] - -14.5417).abs() < 0.01, "got {actions:?}");
    }

    /// Port of legacy `test_forward_only_positive_x`.
    #[test]
    fn test_forward_only_positive_x() {
        let mut controller = HolonomicController::new(test_config()).unwrap();
        let actions = controller.forward([1.0, 0.0, 0.0]);
        let net_x: f64 = controller
            .base_dir
            .iter()
            .zip(&actions)
            .map(|(dir, action)| dir[0] * action)
            .sum();
        assert!(net_x > 0.0, "forward command should produce positive net X velocity");
    }

    /// Port of legacy `test_init_raises_on_missing_required_param`
    /// (empty arrays in Rust instead of None).
    #[test]
    fn test_init_fails_on_missing_required_param() {
        for missing in [
            "wheel_radius",
            "wheel_positions",
            "wheel_orientations",
            "mecanum_angles",
        ] {
            let mut config = test_config();
            match missing {
                "wheel_radius" => config.wheel_radius.clear(),
                "wheel_positions" => config.wheel_positions.clear(),
                "wheel_orientations" => config.wheel_orientations.clear(),
                _ => config.mecanum_angles.clear(),
            }
            assert!(
                HolonomicController::new(config).is_err(),
                "expected error for missing {missing}"
            );
        }
    }

    #[test]
    fn test_scalar_broadcast_and_limits() {
        // Scalar wheel_radius / mecanum_angles broadcast to all wheels
        let mut config = test_config();
        config.wheel_radius = vec![0.04];
        config.mecanum_angles = vec![90.0];
        let mut broadcast = HolonomicController::new(config).unwrap();
        let mut explicit = HolonomicController::new(test_config()).unwrap();
        let command = [1.0, 1.0, 0.1];
        let a = broadcast.forward(command);
        let b = explicit.forward(command);
        for (x, y) in a.iter().zip(&b) {
            assert!((x - y).abs() < 1e-12);
        }

        // max_wheel_speed rescales all wheels proportionally
        let mut config = test_config();
        config.max_wheel_speed = 10.0;
        let mut limited = HolonomicController::new(config).unwrap();
        let limited_actions = limited.forward(command);
        let max = limited_actions.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        assert!((max - 10.0).abs() < 1e-9);
        // Direction is preserved
        for (x, y) in limited_actions.iter().zip(&b) {
            assert!((x / max - y / 25.112).abs() < 0.01);
        }

        // Zero command returns zeros
        assert!(explicit.forward([0.0, 0.0, 0.0]).iter().all(|v| *v == 0.0));
    }
}
