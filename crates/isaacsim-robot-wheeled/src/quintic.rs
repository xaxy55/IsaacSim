// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Quintic polynomials path planner.
//!
//! Port of `controllers/quintic_path_planner.py`, itself derived from Atsushi
//! Sakai's PythonRobotics `quintic_polynomials_planner.py` (MIT license).
//!
//! Ref:
//! - Local Path planning And Motion Control For Agv In Positioning

use isaacsim_core_math::linalg::solve3;

/// Maximum planning horizon in seconds.
pub const MAX_T: f64 = 100.0;

/// Minimum planning horizon in seconds.
pub const MIN_T: f64 = 5.0;

/// Quintic (5th-order) polynomial for one-dimensional trajectory interpolation.
///
/// Solves for coefficients `a0..a5` that satisfy the boundary conditions on
/// position, velocity, and acceleration at times `0` and `time`.
#[derive(Debug, Clone)]
pub struct QuinticPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub a4: f64,
    pub a5: f64,
}

impl QuinticPolynomial {
    /// Solve the boundary-value problem for a segment of duration `time`
    /// seconds with start position/velocity/acceleration `(xs, vxs, axs)` and
    /// end conditions `(xe, vxe, axe)`.
    ///
    /// Returns an error if the system is singular (e.g. `time == 0`; legacy
    /// numpy raises `LinAlgError`).
    pub fn new(
        xs: f64,
        vxs: f64,
        axs: f64,
        xe: f64,
        vxe: f64,
        axe: f64,
        time: f64,
    ) -> Result<Self, String> {
        let a0 = xs;
        let a1 = vxs;
        let a2 = axs / 2.0;

        let t2 = time * time;
        let t3 = t2 * time;
        let t4 = t3 * time;
        let t5 = t4 * time;
        let a = [
            [t3, t4, t5],
            [3.0 * t2, 4.0 * t3, 5.0 * t4],
            [6.0 * time, 12.0 * t2, 20.0 * t3],
        ];
        let b = [
            xe - a0 - a1 * time - a2 * t2,
            vxe - a1 - 2.0 * a2 * time,
            axe - 2.0 * a2,
        ];
        let x = solve3(&a, b)
            .ok_or_else(|| format!("singular boundary-condition system for time={time}"))?;

        Ok(Self {
            a0,
            a1,
            a2,
            a3: x[0],
            a4: x[1],
            a5: x[2],
        })
    }

    /// Evaluate the polynomial (position) at time `t`.
    pub fn calc_point(&self, t: f64) -> f64 {
        self.a0
            + self.a1 * t
            + self.a2 * t.powi(2)
            + self.a3 * t.powi(3)
            + self.a4 * t.powi(4)
            + self.a5 * t.powi(5)
    }

    /// Evaluate the first derivative (velocity) at time `t`.
    pub fn calc_first_derivative(&self, t: f64) -> f64 {
        self.a1
            + 2.0 * self.a2 * t
            + 3.0 * self.a3 * t.powi(2)
            + 4.0 * self.a4 * t.powi(3)
            + 5.0 * self.a5 * t.powi(4)
    }

    /// Evaluate the second derivative (acceleration) at time `t`.
    pub fn calc_second_derivative(&self, t: f64) -> f64 {
        2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t.powi(2) + 20.0 * self.a5 * t.powi(3)
    }

    /// Evaluate the third derivative (jerk) at time `t`.
    pub fn calc_third_derivative(&self, t: f64) -> f64 {
        6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t.powi(2)
    }
}

/// Start or goal state for [`quintic_polynomials_planner`].
#[derive(Debug, Clone, Copy)]
pub struct PlannerState {
    /// X position in m.
    pub x: f64,
    /// Y position in m.
    pub y: f64,
    /// Yaw angle in rad.
    pub yaw: f64,
    /// Velocity in m/s.
    pub v: f64,
    /// Acceleration in m/s^2.
    pub a: f64,
}

/// Planned trajectory returned by [`quintic_polynomials_planner`]
/// (the legacy `(time, rx, ry, ryaw, rv, ra, rj)` tuple).
#[derive(Debug, Clone, Default)]
pub struct QuinticTrajectory {
    /// Sample times in s.
    pub time: Vec<f64>,
    /// X positions in m.
    pub x: Vec<f64>,
    /// Y positions in m.
    pub y: Vec<f64>,
    /// Yaw angles in rad.
    pub yaw: Vec<f64>,
    /// Velocities in m/s.
    pub v: Vec<f64>,
    /// Signed accelerations in m/s^2.
    pub a: Vec<f64>,
    /// Signed jerks in m/s^3.
    pub j: Vec<f64>,
}

/// Plan a trajectory using quintic polynomials between start and goal states.
///
/// Tries planning horizons `T` in `[MIN_T, MAX_T)` (step `MIN_T`) and returns
/// the first trajectory satisfying the `max_accel` (m/s^2) and `max_jerk`
/// (m/s^3) constraints, sampled every `dt` seconds.
///
/// Returns an error if no trajectory satisfies the constraints (legacy raises
/// `ValueError`).
pub fn quintic_polynomials_planner(
    start: PlannerState,
    goal: PlannerState,
    max_accel: f64,
    max_jerk: f64,
    dt: f64,
) -> Result<QuinticTrajectory, String> {
    let vxs = start.v * start.yaw.cos();
    let vys = start.v * start.yaw.sin();
    let vxg = goal.v * goal.yaw.cos();
    let vyg = goal.v * goal.yaw.sin();

    let axs = start.a * start.yaw.cos();
    let ays = start.a * start.yaw.sin();
    let axg = goal.a * goal.yaw.cos();
    let ayg = goal.a * goal.yaw.sin();

    let mut horizon = MIN_T;
    while horizon < MAX_T {
        let xqp = QuinticPolynomial::new(start.x, vxs, axs, goal.x, vxg, axg, horizon)?;
        let yqp = QuinticPolynomial::new(start.y, vys, ays, goal.y, vyg, ayg, horizon)?;

        let mut traj = QuinticTrajectory::default();

        // np.arange(0.0, T + dt, dt): endpoint-inclusive when dt divides T
        let steps = ((horizon + dt) / dt).ceil() as usize;
        for i in 0..steps {
            let t = i as f64 * dt;
            traj.time.push(t);
            traj.x.push(xqp.calc_point(t));
            traj.y.push(yqp.calc_point(t));

            let vx = xqp.calc_first_derivative(t);
            let vy = yqp.calc_first_derivative(t);
            let v = vx.hypot(vy);
            traj.v.push(v);
            traj.yaw.push(vy.atan2(vx));

            let ax = xqp.calc_second_derivative(t);
            let ay = yqp.calc_second_derivative(t);
            let mut a = ax.hypot(ay);
            if traj.v.len() >= 2 && traj.v[traj.v.len() - 1] - traj.v[traj.v.len() - 2] < 0.0 {
                a = -a;
            }
            traj.a.push(a);

            let jx = xqp.calc_third_derivative(t);
            let jy = yqp.calc_third_derivative(t);
            let mut j = jx.hypot(jy);
            if traj.a.len() >= 2 && traj.a[traj.a.len() - 1] - traj.a[traj.a.len() - 2] < 0.0 {
                j = -j;
            }
            traj.j.push(j);
        }

        let max_a = traj.a.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let max_j = traj.j.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if max_a <= max_accel && max_j <= max_jerk {
            return Ok(traj);
        }
        horizon += MIN_T;
    }

    Err(format!(
        "could not find a valid trajectory with max_accel={max_accel}, max_jerk={max_jerk}, \
         and dt={dt} for T in [{MIN_T}, {MAX_T})"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ZERO_STATE: PlannerState = PlannerState {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
        v: 0.0,
        a: 0.0,
    };

    /// Port of legacy `tests/test_quintic_path_planner.py::test_planner_raises_when_constraints_are_exhausted`.
    #[test]
    fn test_planner_fails_when_constraints_are_exhausted() {
        let goal = PlannerState { x: 1.0, ..ZERO_STATE };
        assert!(quintic_polynomials_planner(ZERO_STATE, goal, 0.0, 0.0, 1.0).is_err());
    }

    #[test]
    fn test_planner_reaches_goal() {
        let start = PlannerState {
            x: 10.0,
            y: 10.0,
            yaw: 10.0_f64.to_radians(),
            v: 1.0,
            a: 0.1,
        };
        let goal = PlannerState {
            x: 30.0,
            y: -10.0,
            yaw: 20.0_f64.to_radians(),
            v: 1.0,
            a: 0.1,
        };
        let traj = quintic_polynomials_planner(start, goal, 1.0, 0.5, 0.1).unwrap();

        // Trajectory starts at the start state and ends at the goal state
        assert!((traj.x[0] - start.x).abs() < 1e-9);
        assert!((traj.y[0] - start.y).abs() < 1e-9);
        let last = traj.x.len() - 1;
        assert!((traj.x[last] - goal.x).abs() < 1e-6);
        assert!((traj.y[last] - goal.y).abs() < 1e-6);

        // Constraints hold over the whole trajectory
        assert!(traj.a.iter().all(|a| a.abs() <= 1.0));
        assert!(traj.j.iter().all(|j| j.abs() <= 0.5));
        // All output vectors have the same length
        let n = traj.time.len();
        assert!(n > 0);
        for len in [
            traj.x.len(),
            traj.y.len(),
            traj.yaw.len(),
            traj.v.len(),
            traj.a.len(),
            traj.j.len(),
        ] {
            assert_eq!(len, n);
        }
    }

    #[test]
    fn test_quintic_polynomial_boundary_conditions() {
        let p = QuinticPolynomial::new(0.0, 1.0, 0.5, 10.0, 2.0, -0.5, 5.0).unwrap();
        assert!((p.calc_point(0.0)).abs() < 1e-12);
        assert!((p.calc_first_derivative(0.0) - 1.0).abs() < 1e-12);
        assert!((p.calc_second_derivative(0.0) - 0.5).abs() < 1e-12);
        assert!((p.calc_point(5.0) - 10.0).abs() < 1e-9);
        assert!((p.calc_first_derivative(5.0) - 2.0).abs() < 1e-9);
        assert!((p.calc_second_derivative(5.0) + 0.5).abs() < 1e-9);

        // time = 0 makes the system singular
        assert!(QuinticPolynomial::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).is_err());
    }
}
