// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Path tracking with Stanley steering control and PID speed control.
//!
//! Port of `controllers/stanley_control.py`, itself derived from Atsushi
//! Sakai's PythonRobotics `stanley_controller.py` (MIT license).
//!
//! Ref:
//! - Stanley: The robot that won the DARPA grand challenge
//! - Autonomous Automobile Path Tracking (CMU-RI)

/// Vehicle state for the Stanley controller using a bicycle model.
#[derive(Debug, Clone)]
pub struct State {
    /// Distance between front and rear axles in m.
    pub wheel_base: f64,
    /// X-coordinate in m.
    pub x: f64,
    /// Y-coordinate in m.
    pub y: f64,
    /// Yaw angle in rad.
    pub yaw: f64,
    /// Speed in m/s.
    pub v: f64,
    /// Yaw rate in rad/s (computed by [`Self::update`]).
    pub w: f64,
    /// Maximum steering angle in rad.
    pub max_steering_angle: f64,
}

impl State {
    /// Create a state at the origin with the legacy defaults
    /// (`x = y = yaw = v = 0`, `max_steering_angle = 5 deg`).
    pub fn new(wheel_base: f64) -> Self {
        Self {
            wheel_base,
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            v: 0.0,
            w: 0.0,
            max_steering_angle: 5.0_f64.to_radians(),
        }
    }

    /// Update the vehicle state using the bicycle kinematic model.
    ///
    /// `acceleration` is the longitudinal acceleration in m/s^2, `delta` the
    /// steering angle in rad (clamped to `max_steering_angle`), `dt` the time
    /// step in s.
    pub fn update(&mut self, acceleration: f64, delta: f64, dt: f64) {
        let delta = delta.clamp(-self.max_steering_angle, self.max_steering_angle);
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.w = self.v / self.wheel_base * delta.tan();
        self.yaw = normalize_angle(self.yaw + self.w * dt)
            .expect("yaw became non-finite; check acceleration/dt inputs");
        self.v += acceleration * dt;
    }
}

/// Compute proportional control output for speed tracking
/// (`kp * (target - current)`; legacy default `kp = 0.1`).
pub fn pid_control(target: f64, current: f64, kp: f64) -> f64 {
    kp * (target - current)
}

/// Compute the Stanley steering control output.
///
/// `cx`/`cy`/`cyaw` describe the reference path, `last_target_idx` is the
/// previous target index on the path, and `k` is the cross-track error gain
/// (legacy default `0.5`; the legacy `p`/`i`/`d` parameters are unused there
/// and omitted here).
///
/// Returns `(steering_angle, target_index)`, or an error if the path is empty
/// or an angle is non-finite.
pub fn stanley_control(
    state: &State,
    cx: &[f64],
    cy: &[f64],
    cyaw: &[f64],
    last_target_idx: usize,
    k: f64,
) -> Result<(f64, usize), String> {
    let (current_target_idx, error_front_axle) = calc_target_index(state, cx, cy)?;
    let current_target_idx = current_target_idx.max(last_target_idx);

    let theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)?;
    let theta_d = (k * normalize_angle(error_front_axle)?).atan2(state.v);

    Ok((theta_e + theta_d, current_target_idx))
}

/// Normalize an angle to `[-pi, pi]`.
///
/// Returns an error if the angle is not finite (legacy raises `ValueError`).
pub fn normalize_angle(angle: f64) -> Result<f64, String> {
    use std::f64::consts::PI;
    if !angle.is_finite() {
        return Err("angle must be finite".to_string());
    }
    let normalized = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if normalized == -PI && angle > 0.0 {
        Ok(PI)
    } else {
        Ok(normalized)
    }
}

/// Compute the nearest target index on the trajectory and the cross-track
/// error at the front axle.
pub fn calc_target_index(state: &State, cx: &[f64], cy: &[f64]) -> Result<(usize, f64), String> {
    if cx.is_empty() || cx.len() != cy.len() {
        return Err(format!(
            "cx and cy must be non-empty and the same length, got {} and {}",
            cx.len(),
            cy.len()
        ));
    }
    let fx = state.x + state.wheel_base * state.yaw.cos();
    let fy = state.y + state.wheel_base * state.yaw.sin();

    let mut target_idx = 0;
    let mut min_d = f64::INFINITY;
    for (i, (icx, icy)) in cx.iter().zip(cy).enumerate() {
        let d = (fx - icx).hypot(fy - icy);
        if d < min_d {
            min_d = d;
            target_idx = i;
        }
    }

    let front_axle_vec = [
        -(state.yaw + std::f64::consts::FRAC_PI_2).cos(),
        -(state.yaw + std::f64::consts::FRAC_PI_2).sin(),
    ];
    let error_front_axle =
        (fx - cx[target_idx]) * front_axle_vec[0] + (fy - cy[target_idx]) * front_axle_vec[1];

    Ok((target_idx, error_front_axle))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Port of legacy `tests/test_stanley_control.py::test_normalize_angle_maps_finite_angles_to_pi_range`.
    #[test]
    fn test_normalize_angle_maps_finite_angles_to_pi_range() {
        assert!((normalize_angle(5.0 * PI).unwrap() - PI).abs() < 1e-12);
        assert!((normalize_angle(-5.0 * PI).unwrap() + PI).abs() < 1e-12);
        assert!((normalize_angle(1.5 * PI).unwrap() + 0.5 * PI).abs() < 1e-12);
    }

    /// Port of legacy `test_normalize_angle_rejects_non_finite_angles`.
    #[test]
    fn test_normalize_angle_rejects_non_finite_angles() {
        assert!(normalize_angle(f64::INFINITY).is_err());
        assert!(normalize_angle(f64::NEG_INFINITY).is_err());
        assert!(normalize_angle(f64::NAN).is_err());
    }

    #[test]
    fn test_stanley_control_tracks_straight_path() {
        // Vehicle slightly left of a straight path along +X should steer right
        // (negative steering angle) and pick a monotonically advancing index.
        let cx: Vec<f64> = (0..50).map(|i| i as f64 * 0.1).collect();
        let cy = vec![0.0; 50];
        let cyaw = vec![0.0; 50];

        let mut state = State::new(0.5);
        state.y = 0.5;
        state.v = 1.0;

        let (delta, idx) = stanley_control(&state, &cx, &cy, &cyaw, 0, 0.5).unwrap();
        assert!(delta < 0.0, "expected right steering, got {delta}");
        assert!(idx < cx.len());

        // Target index never goes backwards
        let (_, idx2) = stanley_control(&state, &cx, &cy, &cyaw, idx + 3, 0.5).unwrap();
        assert!(idx2 >= idx + 3);
    }

    #[test]
    fn test_state_update_bicycle_model() {
        let mut state = State::new(1.0);
        state.v = 1.0;
        state.max_steering_angle = PI / 4.0;
        state.update(0.0, PI / 4.0, 0.1);
        assert!((state.x - 0.1).abs() < 1e-12);
        assert_eq!(state.y, 0.0);
        // w = v / L * tan(delta) = 1.0
        assert!((state.w - 1.0).abs() < 1e-12);
        assert!((state.yaw - 0.1).abs() < 1e-12);

        // Steering angle is clamped to max_steering_angle
        let mut clamped = State::new(1.0);
        clamped.v = 1.0;
        clamped.update(0.0, PI, 0.1); // clamped to 5 deg
        assert!((clamped.w - 5.0_f64.to_radians().tan()).abs() < 1e-12);
    }

    #[test]
    fn test_pid_control() {
        assert!((pid_control(2.0, 1.0, 0.1) - 0.1).abs() < 1e-12);
    }
}
