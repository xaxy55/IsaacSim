// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! In-memory scene representation for Isaac Sim.
//!
//! Phase 2 foundation crate: a native Rust stage/prim data model exposing the
//! subset of USD semantics that the ported Isaac Sim logic layer needs (prim
//! hierarchy, typed attributes, inherit composition). It is the in-memory
//! backend; a real USD backend (FFI or `openusd-rs`) can be bound behind the
//! same API later, per the strategy in ROADMAP.md.

pub mod backend;
pub mod path;
pub mod stage;
pub mod usda;
pub mod value;
pub mod xform;

pub use backend::{descendants, SceneStage};
pub use path::{is_valid_path_string, parent_path};
pub use stage::{Prim, Stage, UpAxis};
pub use usda::{parse_usda, write_usda};
pub use value::Value;
