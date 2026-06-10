// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Ackermann steering controller for wheeled robots.
//!
//! Port of `controllers/ackermann_controller.py`.

/// Input command for [`AckermannController::forward`].
#[derive(Debug, Clone, Copy)]
pub struct AckermannCommand {
    /// Desired steering angle in rad.
    pub steering_angle: f64,
    /// Steering rate in rad/s (0 to snap directly to the target angle).
    pub steering_angle_velocity: f64,
    /// Desired forward speed in m/s.
    pub speed: f64,
    /// Linear acceleration in m/s^2 (0 to snap directly to the target speed).
    pub acceleration: f64,
    /// Time step in s.
    pub dt: f64,
}

/// Output of [`AckermannController::forward`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AckermannOutput {
    /// Steering joint positions: (left_wheel_angle, right_wheel_angle) in rad.
    pub joint_positions: (f64, f64),
    /// Wheel angular velocities: (front-left, front-right, back-left, back-right) in rad/s.
    pub joint_velocities: (f64, f64, f64, f64),
}

/// Ackermann steering controller using a bicycle model.
///
/// Computes left and right steering angles and per-wheel rotation velocities
/// for a four-wheel Ackermann robot.
#[derive(Debug, Clone)]
pub struct AckermannController {
    /// Distance between front and rear axles in m.
    pub wheel_base: f64,
    /// Distance between left and right wheels in m.
    pub track_width: f64,
    /// Radius of front wheels in m.
    pub front_wheel_radius: f64,
    /// Radius of back wheels in m.
    pub back_wheel_radius: f64,
    /// Maximum angular velocity of wheels in rad/s. Ignored if 0.
    pub max_wheel_velocity: f64,
    /// True for rear wheel steering.
    pub invert_steering: bool,
    /// Maximum steering angle in rad. Ignored if 0.
    pub max_wheel_rotation_angle: f64,
    /// Maximum linear acceleration in m/s^2. Ignored if 0.
    pub max_acceleration: f64,
    /// Maximum steering rate in rad/s. Ignored if 0.
    pub max_steering_angle_velocity: f64,

    prev_linear_velocity: f64,
    prev_steering_angle: f64,
}

impl AckermannController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        wheel_base: f64,
        track_width: f64,
        front_wheel_radius: f64,
        back_wheel_radius: f64,
        max_wheel_velocity: f64,
        invert_steering: bool,
        max_wheel_rotation_angle: f64,
        max_acceleration: f64,
        max_steering_angle_velocity: f64,
    ) -> Self {
        Self {
            wheel_base: wheel_base.abs(),
            track_width: track_width.abs(),
            front_wheel_radius: front_wheel_radius.abs(),
            back_wheel_radius: back_wheel_radius.abs(),
            max_wheel_velocity: max_wheel_velocity.abs(),
            invert_steering,
            max_wheel_rotation_angle: max_wheel_rotation_angle.abs(),
            max_acceleration: max_acceleration.abs(),
            max_steering_angle_velocity: max_steering_angle_velocity.abs(),
            prev_linear_velocity: 0.0,
            prev_steering_angle: 0.0,
        }
    }

    /// Compute wheel angles and wheel rotation velocities from an Ackermann
    /// command. Returns `None` on invalid input (legacy returns `(None, None)`).
    pub fn forward(&mut self, command: AckermannCommand) -> Option<AckermannOutput> {
        let mut fwr = self.front_wheel_radius;
        let mut bwr = self.back_wheel_radius;
        if fwr == 0.0 && bwr == 0.0 {
            return None;
        }
        if fwr == 0.0 {
            fwr = bwr;
        }
        if bwr == 0.0 {
            bwr = fwr;
        }

        let max_wv = if self.max_wheel_velocity > 0.0 {
            self.max_wheel_velocity
        } else {
            f64::INFINITY
        };
        let max_rot = if self.max_wheel_rotation_angle > 0.0 {
            self.max_wheel_rotation_angle
        } else {
            f64::INFINITY
        };
        let max_acc = if self.max_acceleration > 0.0 {
            self.max_acceleration
        } else {
            f64::INFINITY
        };
        let max_steer_vel = if self.max_steering_angle_velocity > 0.0 {
            self.max_steering_angle_velocity
        } else {
            f64::INFINITY
        };

        let effective_radius = fwr.max(bwr);
        let max_linear_velocity = (max_wv * effective_radius).abs();

        let target_angle = command.steering_angle.clamp(-max_rot, max_rot);
        let target_speed = command.speed.clamp(-max_linear_velocity, max_linear_velocity);
        let acceleration = if max_acc.is_finite() {
            command.acceleration.abs().min(max_acc)
        } else {
            command.acceleration
        };
        let steering_velocity = if max_steer_vel.is_finite() {
            command.steering_angle_velocity.abs().min(max_steer_vel)
        } else {
            command.steering_angle_velocity
        };
        let dt = command.dt.abs();

        if dt == 0.0 && (steering_velocity != 0.0 || acceleration != 0.0) {
            return None;
        }

        let mut forward_vel = self.prev_linear_velocity;
        if acceleration == 0.0 {
            forward_vel = target_speed;
        } else {
            let velocity_diff = target_speed - self.prev_linear_velocity;
            if velocity_diff.abs() > 0.0001 {
                if velocity_diff > 0.0 {
                    forward_vel = (self.prev_linear_velocity + acceleration * dt).min(target_speed);
                } else {
                    forward_vel = (self.prev_linear_velocity - acceleration * dt).max(target_speed);
                }
            }
        }
        self.prev_linear_velocity = forward_vel;

        let mut steering_angle = self.prev_steering_angle;
        if steering_velocity == 0.0 {
            steering_angle = target_angle;
        } else {
            let steering_angle_diff = target_angle - self.prev_steering_angle;
            if steering_angle_diff.abs() > 0.00174533 {
                if steering_angle_diff > 0.0 {
                    steering_angle =
                        (self.prev_steering_angle + steering_velocity * dt).min(target_angle);
                } else {
                    steering_angle =
                        (self.prev_steering_angle - steering_velocity * dt).max(target_angle);
                }
            }
        }
        self.prev_steering_angle = steering_angle;

        let (left_wheel_angle, right_wheel_angle, v_fl, v_fr, v_bl, v_br);
        if steering_angle.abs() < 0.0157 {
            left_wheel_angle = 0.0;
            right_wheel_angle = 0.0;
            v_fl = forward_vel / fwr;
            v_fr = forward_vel / fwr;
            v_bl = forward_vel / bwr;
            v_br = forward_vel / bwr;
        } else {
            let steer_sign = if self.invert_steering { -1.0 } else { 1.0 };
            let r = steer_sign * self.wheel_base / steering_angle.tan();
            left_wheel_angle = (self.wheel_base / (r - 0.5 * self.track_width)).atan();
            right_wheel_angle = (self.wheel_base / (r + 0.5 * self.track_width)).atan();

            let steering_joint_half_dist = self.track_width / 2.0;
            let cy = r.abs();
            let sign = if steering_angle > 0.0 { 1.0 } else { -1.0 };

            let (wheel_dist_fl, wheel_dist_fr, wheel_dist_bl, wheel_dist_br);
            if self.invert_steering {
                wheel_dist_fl = cy - sign * steering_joint_half_dist;
                wheel_dist_fr = cy + sign * steering_joint_half_dist;
                wheel_dist_bl = ((cy - sign * steering_joint_half_dist).powi(2)
                    + self.wheel_base.powi(2))
                .sqrt();
                wheel_dist_br = ((cy + sign * steering_joint_half_dist).powi(2)
                    + self.wheel_base.powi(2))
                .sqrt();
            } else {
                wheel_dist_fl = ((cy - sign * steering_joint_half_dist).powi(2)
                    + self.wheel_base.powi(2))
                .sqrt();
                wheel_dist_fr = ((cy + sign * steering_joint_half_dist).powi(2)
                    + self.wheel_base.powi(2))
                .sqrt();
                wheel_dist_bl = cy - sign * steering_joint_half_dist;
                wheel_dist_br = cy + sign * steering_joint_half_dist;
            }

            let body_ang_vel = forward_vel / cy;
            v_fl = body_ang_vel * (wheel_dist_fl / fwr);
            v_fr = body_ang_vel * (wheel_dist_fr / fwr);
            v_bl = body_ang_vel * (wheel_dist_bl / bwr);
            v_br = body_ang_vel * (wheel_dist_br / bwr);
        }

        Some(AckermannOutput {
            joint_positions: (
                left_wheel_angle.clamp(-max_rot, max_rot),
                right_wheel_angle.clamp(-max_rot, max_rot),
            ),
            joint_velocities: (
                v_fl.clamp(-max_wv, max_wv),
                v_fr.clamp(-max_wv, max_wv),
                v_bl.clamp(-max_wv, max_wv),
                v_br.clamp(-max_wv, max_wv),
            ),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn controller() -> AckermannController {
        AckermannController::new(1.65, 1.25, 0.25, 0.25, 0.0, false, 0.0, 0.0, 0.0)
    }

    fn cmd(
        steering_angle: f64,
        steering_angle_velocity: f64,
        speed: f64,
        acceleration: f64,
        dt: f64,
    ) -> AckermannCommand {
        AckermannCommand {
            steering_angle,
            steering_angle_velocity,
            speed,
            acceleration,
            dt,
        }
    }

    #[test]
    fn test_straight_drive() {
        let mut c = controller();
        let out = c.forward(cmd(0.0, 0.0, 1.0, 0.0, 1.0 / 60.0)).unwrap();
        assert_eq!(out.joint_positions, (0.0, 0.0));
        let v = 1.0 / 0.25;
        assert_eq!(out.joint_velocities, (v, v, v, v));
    }

    #[test]
    fn test_zero_wheel_radius_invalid() {
        let mut c = AckermannController::new(1.65, 1.25, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0);
        assert!(c.forward(cmd(0.0, 0.0, 1.0, 0.0, 0.1)).is_none());
    }

    #[test]
    fn test_zero_dt_with_rates_invalid() {
        let mut c = controller();
        assert!(c.forward(cmd(0.2, 0.5, 1.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn test_turning_inner_wheel_steers_sharper() {
        let mut c = controller();
        let out = c.forward(cmd(0.3, 0.0, 1.0, 0.0, 1.0 / 60.0)).unwrap();
        let (left, right) = out.joint_positions;
        // Left turn: left (inner) wheel angle is larger than right (outer).
        assert!(left > right, "left={left} right={right}");
        assert!(left > 0.0 && right > 0.0);
        // Outer wheels travel farther, so they spin faster.
        let (v_fl, v_fr, v_bl, v_br) = out.joint_velocities;
        assert!(v_fr > v_fl);
        assert!(v_br > v_bl);
    }

    #[test]
    fn test_acceleration_ramps_speed() {
        let mut c = controller();
        // 1 m/s^2 for 0.1 s from standstill toward 1 m/s -> 0.1 m/s.
        let out = c.forward(cmd(0.0, 0.0, 1.0, 1.0, 0.1)).unwrap();
        let expected = 0.1 / 0.25;
        let (v_fl, ..) = out.joint_velocities;
        assert!((v_fl - expected).abs() < 1e-12, "v_fl={v_fl}");
    }

    #[test]
    fn test_wheel_velocity_limit() {
        let mut c = AckermannController::new(1.65, 1.25, 0.25, 0.25, 2.0, false, 0.0, 0.0, 0.0);
        let out = c.forward(cmd(0.0, 0.0, 10.0, 0.0, 0.1)).unwrap();
        let (v_fl, v_fr, v_bl, v_br) = out.joint_velocities;
        for v in [v_fl, v_fr, v_bl, v_br] {
            assert!(v <= 2.0);
        }
    }
}
