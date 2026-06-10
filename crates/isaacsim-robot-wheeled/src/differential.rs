// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Differential drive controller for wheeled robots.
//!
//! Port of `controllers/differential_controller.py`.

/// Unicycle differential drive controller.
///
/// Converts `[linear_speed, angular_speed]` commands into `[left, right]`
/// wheel velocities using the standard differential-drive kinematic model.
#[derive(Debug, Clone)]
pub struct DifferentialController {
    /// Radius of each drive wheel in m.
    pub wheel_radius: f64,
    /// Distance between left and right wheels in m.
    pub wheel_base: f64,
    /// Maximum forward/backward speed in m/s.
    pub max_linear_speed: f64,
    /// Maximum yaw rate in rad/s.
    pub max_angular_speed: f64,
    /// Maximum individual wheel angular velocity in rad/s.
    pub max_wheel_speed: f64,
}

impl DifferentialController {
    /// Create a controller with no speed limits (legacy default of `1e20`).
    ///
    /// # Panics
    ///
    /// Does not panic; limits default to `f64::INFINITY`. Use
    /// [`Self::try_new`] to validate explicit limits.
    pub fn new(wheel_radius: f64, wheel_base: f64) -> Self {
        Self {
            wheel_radius,
            wheel_base,
            max_linear_speed: f64::INFINITY,
            max_angular_speed: f64::INFINITY,
            max_wheel_speed: f64::INFINITY,
        }
    }

    /// Create a controller with explicit speed limits.
    ///
    /// Returns an error if any limit is negative (legacy raises `ValueError`).
    pub fn try_new(
        wheel_radius: f64,
        wheel_base: f64,
        max_linear_speed: f64,
        max_angular_speed: f64,
        max_wheel_speed: f64,
    ) -> Result<Self, String> {
        if max_linear_speed < 0.0 {
            return Err(format!("max_linear_speed must be >= 0, got {max_linear_speed}"));
        }
        if max_angular_speed < 0.0 {
            return Err(format!("max_angular_speed must be >= 0, got {max_angular_speed}"));
        }
        if max_wheel_speed < 0.0 {
            return Err(format!("max_wheel_speed must be >= 0, got {max_wheel_speed}"));
        }
        Ok(Self {
            wheel_radius,
            wheel_base,
            max_linear_speed,
            max_angular_speed,
            max_wheel_speed,
        })
    }

    /// Convert `[linear_speed, angular_speed]` to `[left_wheel, right_wheel]`
    /// angular velocities in rad/s.
    pub fn forward(&self, command: [f64; 2]) -> [f64; 2] {
        let linear = command[0].clamp(-self.max_linear_speed, self.max_linear_speed);
        let angular = command[1].clamp(-self.max_angular_speed, self.max_angular_speed);
        // omega_L = (2V - omega*b)/(2r), omega_R = (2V + omega*b)/(2r)
        let left = (2.0 * linear - angular * self.wheel_base) / (2.0 * self.wheel_radius);
        let right = (2.0 * linear + angular * self.wheel_base) / (2.0 * self.wheel_radius);
        [
            left.clamp(-self.max_wheel_speed, self.max_wheel_speed),
            right.clamp(-self.max_wheel_speed, self.max_wheel_speed),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Port of legacy `tests/test_differential_controller.py::test_differential_drive`.
    #[test]
    fn test_differential_drive() {
        let mut controller = DifferentialController::new(0.03, 0.1125);

        let command = [0.3, 1.0];
        assert_eq!(controller.forward(command), [8.125, 11.875]);

        controller.max_wheel_speed = 9.0;
        assert_eq!(controller.forward(command), [8.125, 9.0]);
    }

    #[test]
    fn test_negative_limits_rejected() {
        assert!(DifferentialController::try_new(0.03, 0.1125, -1.0, 1.0, 1.0).is_err());
        assert!(DifferentialController::try_new(0.03, 0.1125, 1.0, -1.0, 1.0).is_err());
        assert!(DifferentialController::try_new(0.03, 0.1125, 1.0, 1.0, -1.0).is_err());
    }

    #[test]
    fn test_command_clamped_before_kinematics() {
        let controller =
            DifferentialController::try_new(0.03, 0.1125, 0.2, 0.5, f64::INFINITY).unwrap();
        let unclamped = DifferentialController::new(0.03, 0.1125);
        assert_eq!(controller.forward([0.3, 1.0]), unclamped.forward([0.2, 0.5]));
    }
}
