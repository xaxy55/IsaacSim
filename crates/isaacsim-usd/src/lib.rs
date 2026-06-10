// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! OpenUSD backend for Isaac Sim.
//!
//! Implements [`isaacsim_scene::SceneStage`] over the C++ OpenUSD library
//! via a C ABI shim (`src/shim.cpp`), so the ported logic layer (e.g.
//! `isaacsim-core-cloner`) runs unchanged on real USD stages.
//!
//! Enable the `openusd` feature and point `USD_ROOT` at an OpenUSD install
//! prefix (default `/opt/openusd`). Without the feature this crate is an
//! empty placeholder so the workspace builds in environments without USD.

#[cfg(feature = "openusd")]
mod usd_stage;

#[cfg(feature = "openusd")]
pub use usd_stage::UsdStage;
