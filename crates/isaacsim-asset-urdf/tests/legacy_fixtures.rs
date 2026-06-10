// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Parse the real URDF fixtures shipped with the legacy
//! `isaacsim.asset.importer.urdf` extension.

use isaacsim_asset_urdf::{
    merge_fixed_joints, parse_urdf, parse_urdf_file, write_urdf, Geometry, JointType,
};

fn fixture(name: &str) -> String {
    format!(
        "{}/../../legacy/source/extensions/isaacsim.asset.importer.urdf/data/urdf/tests/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

#[test]
fn test_parse_test_mimic_fixture() {
    let robot = parse_urdf_file(fixture("test_mimic.urdf")).unwrap();
    assert_eq!(robot.name, "test_mimic");
    assert_eq!(robot.links.len(), 4);
    assert_eq!(robot.joints.len(), 3);

    let source = robot.joint("source_joint").unwrap();
    assert_eq!(source.joint_type, JointType::Revolute);
    assert_eq!(source.axis, [0.0, 0.0, 1.0]);
    let limit = source.limit.unwrap();
    assert_eq!(limit.lower, -1.57);
    assert_eq!(limit.upper, 1.57);
    assert_eq!(limit.effort, 100.0);
    assert!(source.mimic.is_none());

    // Mimic joints regardless of lexicographic order vs the source joint
    let a = robot.joint("a_mimic_joint").unwrap().mimic.as_ref().unwrap();
    assert_eq!(a.joint, "source_joint");
    assert_eq!(a.multiplier, 1.5);
    assert_eq!(a.offset, 0.1);
    let z = robot.joint("z_mimic_joint").unwrap().mimic.as_ref().unwrap();
    assert_eq!(z.joint, "source_joint");
    assert_eq!(z.multiplier, -1.0);
    assert_eq!(z.offset, 0.0);

    // base_link has box visual/collision and an inertial
    let base = robot.link("base_link").unwrap();
    assert!(matches!(base.visuals[0].geometry, Geometry::Box { size } if size == [0.1, 0.1, 0.1]));
    assert_eq!(base.collisions.len(), 1);
    assert_eq!(base.inertial.unwrap().mass, 1.0);
}

#[test]
fn test_parse_test_limits_fixture_and_merge() {
    let mut robot = parse_urdf_file(fixture("test_limits.urdf")).unwrap();
    assert_eq!(robot.name, "test_limits");

    // The fixture has a fixed root_to_base joint with no <origin>
    let fixed = robot.joint("root_to_base").unwrap();
    assert_eq!(fixed.joint_type, JointType::Fixed);
    assert_eq!(fixed.origin.xyz, [0.0, 0.0, 0.0]);

    let elbow = robot.joint("elbow_joint").unwrap();
    let limit = elbow.limit.unwrap();
    assert_eq!(limit.lower, -0.6);
    assert_eq!(limit.upper, 0.6);
    assert_eq!(limit.effort, 1000.0);
    assert_eq!(limit.velocity, -1.0);
    assert_eq!(elbow.origin.rpy[0], 1.571);

    let link1 = robot.link("link_1").unwrap();
    assert!(
        matches!(link1.visuals[0].geometry, Geometry::Cylinder { radius, length }
            if radius == 0.1 && length == 0.8)
    );

    // Merging collapses root_link <- base_link into a single root
    let links_before = robot.links.len();
    merge_fixed_joints(&mut robot);
    assert!(robot.joint("root_to_base").is_none());
    assert!(robot.link("base_link").is_none());
    let root = robot.link("root_link").unwrap();
    assert!(!root.visuals.is_empty());
    assert_eq!(robot.links.len(), links_before - 1);
}

#[test]
fn test_write_round_trip() {
    let original = parse_urdf_file(fixture("test_mimic.urdf")).unwrap();
    let round_tripped = parse_urdf(&write_urdf(&original)).unwrap();
    assert_eq!(original, round_tripped);
}
