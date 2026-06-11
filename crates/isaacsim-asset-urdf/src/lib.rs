// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! URDF parsing and pre-processing for Isaac Sim.
//!
//! Rust port of the parsing layer of the legacy `isaacsim.asset.importer.urdf`
//! extension: URDF XML → [`Robot`](model::Robot) model, the
//! [`merge_fixed_joints`] pre-processing pass (transform composition and
//! parallel-axis inertia merging from `python/impl/urdf_utils.py`), and a
//! URDF writer for the file-to-file flow, plus a rigid-body
//! URDF → stage converter ([`convert_urdf_to_stage`]) targeting any
//! `isaacsim-scene` backend. Mesh tessellation and PhysX runtime setup stay
//! downstream (see ROADMAP.md).

pub mod convert;
pub mod merge;
pub mod model;
pub mod parse;
pub mod transform;
pub mod write;

pub use convert::convert_urdf_to_stage;
pub use merge::merge_fixed_joints;
pub use model::{
    Collision, Geometry, Inertia, Inertial, Joint, JointCalibration, JointDynamics, JointLimit,
    JointMimic, JointType, Link, Material, Origin, Robot, SafetyController, Visual,
};
pub use parse::{parse_urdf, parse_urdf_file};
pub use write::write_urdf;

/// File-to-file pre-processing like the legacy
/// `merge_fixed_joints(urdf_path, output_path)`: parse, merge, write.
///
/// Returns the output path. Note: non-standard extension elements
/// (`<transmission>`, `<gazebo>`, …) are not carried through.
pub fn merge_fixed_joints_file(
    urdf_path: impl AsRef<std::path::Path>,
    output_path: impl AsRef<std::path::Path>,
) -> Result<std::path::PathBuf, String> {
    let mut robot = parse_urdf_file(urdf_path)?;
    merge_fixed_joints(&mut robot);
    let output = output_path.as_ref().to_path_buf();
    std::fs::write(&output, write_urdf(&robot))
        .map_err(|e| format!("failed to write {}: {e}", output.display()))?;
    Ok(output)
}
