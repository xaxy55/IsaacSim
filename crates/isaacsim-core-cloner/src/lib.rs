// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Object duplication APIs for Isaac Sim.
//!
//! Rust port of the legacy `isaacsim.core.cloner` extension
//! (`legacy/source/extensions/isaacsim.core.cloner`), operating on the
//! in-memory [`isaacsim_scene::Stage`]. The PhysX replication and Fabric
//! cloning paths depend on closed-source components and are out of scope;
//! see ROADMAP.md.

mod cloner;
mod grid_cloner;

pub use cloner::{CloneOptions, Cloner};
pub use grid_cloner::GridCloner;
