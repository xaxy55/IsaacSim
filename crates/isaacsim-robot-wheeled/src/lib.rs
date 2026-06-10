// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Wheeled-robot controllers for Isaac Sim.
//!
//! Rust port of the legacy `isaacsim.robot.experimental.wheeled_robots`
//! extension. Currently ported: [`DifferentialController`],
//! [`AckermannController`]. Remaining (see ROADMAP.md): holonomic controller,
//! Stanley control, quintic path planner.

mod ackermann;
mod differential;

pub use ackermann::{AckermannCommand, AckermannController, AckermannOutput};
pub use differential::DifferentialController;
