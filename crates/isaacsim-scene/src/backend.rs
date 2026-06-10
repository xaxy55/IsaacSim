// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Stage backend abstraction.
//!
//! [`SceneStage`] is the surface the ported Isaac Sim logic layer uses to
//! talk to a scene. The in-memory [`Stage`](crate::Stage) implements it
//! natively; the `isaacsim-usd` crate implements it over OpenUSD via FFI,
//! so ported code (e.g. the cloner) runs unchanged on real USD stages.
//!
//! Reads are *composed* queries (USD semantics): they resolve through
//! inherit arcs, not just locally authored opinions.

use crate::stage::UpAxis;
use crate::value::Value;

/// A scene stage: prim hierarchy, typed attributes, inherit arcs.
pub trait SceneStage {
    /// Stage up axis.
    fn up_axis(&self) -> UpAxis;

    /// Set the stage up axis.
    fn set_up_axis(&mut self, up_axis: UpAxis);

    /// Define a prim at `path` (creating missing ancestors); a non-empty
    /// `type_name` overrides the prim's type.
    fn define_prim(&mut self, path: &str, type_name: &str) -> Result<(), String>;

    /// Whether a prim exists at `path` in the composed stage.
    fn prim_exists(&self, path: &str) -> bool;

    /// The composed type name of the prim at `path`; `None` if the prim
    /// does not exist.
    fn type_name(&self, path: &str) -> Option<String>;

    /// The composed value of attribute `name` on the prim at `path`.
    fn attribute(&self, path: &str, name: &str) -> Option<Value>;

    /// Author attribute `name` on the prim at `path`.
    fn set_attribute(&mut self, path: &str, name: &str, value: Value) -> Result<(), String>;

    /// Remove the authored attribute `name` from the prim at `path`.
    /// Removing a non-existent attribute is not an error.
    fn remove_attribute(&mut self, path: &str, name: &str) -> Result<(), String>;

    /// Prepend `source` to the inherit list of the prim at `path`.
    fn add_inherit(&mut self, path: &str, source: &str) -> Result<(), String>;

    /// Deep-copy the prim subtree at `source` to `dest` (`Sdf.CopySpec`).
    fn copy_spec(&mut self, source: &str, dest: &str) -> Result<(), String>;
}
