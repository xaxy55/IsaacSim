// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Wheeled-robot controllers for Isaac Sim.
//!
//! Rust port of the controllers in the legacy
//! `isaacsim.robot.experimental.wheeled_robots` extension:
//! [`DifferentialController`], [`AckermannController`],
//! [`HolonomicController`], Stanley steering control ([`stanley`]), and the
//! quintic polynomials path planner ([`quintic`]), plus the
//! [`HolonomicRobotUsdSetup`] stage reader from `robots/`. The legacy
//! `WheeledRobot` runtime wrapper (articulation control) depends on the
//! simulation core and is not ported.

mod ackermann;
mod differential;
mod holonomic;
pub mod quintic;
pub mod stanley;
pub mod usd_setup;

pub use ackermann::{AckermannCommand, AckermannController, AckermannOutput};
pub use differential::DifferentialController;
pub use holonomic::{HolonomicConfig, HolonomicController};
pub use quintic::{quintic_polynomials_planner, PlannerState, QuinticPolynomial, QuinticTrajectory};
pub use stanley::{calc_target_index, normalize_angle, pid_control, stanley_control, State};
pub use usd_setup::HolonomicRobotUsdSetup;
