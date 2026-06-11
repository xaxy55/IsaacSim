// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! The URDF → stage converter writing directly to a real OpenUSD stage:
//! parse a legacy fixture, author it through the FFI backend, and verify
//! the composed result through real USD queries.

#![cfg(feature = "openusd")]

use isaacsim_asset_urdf::{
    convert_urdf_to_stage, export_stage_to_urdf, merge_fixed_joints, parse_urdf_file, JointType,
};
use isaacsim_scene::{SceneStage, Value};
use isaacsim_usd::UsdStage;

fn fixture(name: &str) -> String {
    format!(
        "{}/../../legacy/source/extensions/isaacsim.asset.importer.urdf/data/urdf/tests/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

#[test]
fn test_urdf_to_real_usd_stage() {
    let mut robot = parse_urdf_file(fixture("test_limits.urdf")).unwrap();
    merge_fixed_joints(&mut robot);

    let mut stage = UsdStage::create_in_memory().unwrap();
    convert_urdf_to_stage(&robot, &mut stage, "/test_limits").unwrap();

    // Applied schemas visible through real USD composition
    assert!(stage.has_api_schema("/test_limits", "PhysicsArticulationRootAPI"));
    assert!(stage.has_api_schema("/test_limits/link_1", "PhysicsRigidBodyAPI"));
    assert!(stage.has_api_schema("/test_limits/link_1", "PhysicsMassAPI"));
    assert!(!stage.has_api_schema("/test_limits/link_1", "PhysicsArticulationRootAPI"));

    // Joint authored as a real UsdPhysics revolute joint
    let elbow = "/test_limits/joints/elbow_joint";
    assert_eq!(stage.type_name(elbow).unwrap(), "PhysicsRevoluteJoint");
    assert_eq!(
        stage.relationship_targets(elbow, "physics:body0"),
        Some(vec!["/test_limits/link_1".to_string()])
    );
    let Some(Value::Float(lower)) = stage.attribute(elbow, "physics:lowerLimit") else {
        panic!("missing lower limit");
    };
    assert!((lower - (-0.6f32).to_degrees()).abs() < 1e-3);

    // Export from real USD parses back through the in-memory subset reader
    let exported = stage.export_to_string().unwrap();
    let parsed = isaacsim_scene::Stage::from_usda(&exported).unwrap();
    assert!(parsed.has_api_schema("/test_limits/link_1", "PhysicsRigidBodyAPI"));
    assert_eq!(
        parsed.relationship_targets(elbow, "physics:body1"),
        Some(vec!["/test_limits/link_2".to_string()])
    );

    // Full round trip: read the real USD stage back into a URDF robot model
    let round_tripped = export_stage_to_urdf(&stage, "/test_limits", "test_limits").unwrap();
    assert_eq!(round_tripped.links.len(), robot.links.len());
    assert_eq!(round_tripped.joints.len(), robot.joints.len());
    let rt_elbow = round_tripped.joint("elbow_joint").unwrap();
    let orig_elbow = robot.joint("elbow_joint").unwrap();
    assert_eq!(rt_elbow.joint_type, JointType::Revolute);
    assert_eq!(rt_elbow.parent, orig_elbow.parent);
    assert_eq!(rt_elbow.child, orig_elbow.child);
    let (rt_limit, orig_limit) = (rt_elbow.limit.unwrap(), orig_elbow.limit.unwrap());
    assert!((rt_limit.lower - orig_limit.lower).abs() < 1e-6);
    assert!((rt_limit.upper - orig_limit.upper).abs() < 1e-6);
    for (a, b) in rt_elbow.axis.iter().zip(&orig_elbow.axis) {
        assert!((a - b).abs() < 1e-6);
    }
    assert_eq!(
        round_tripped.joint("base_joint").unwrap().joint_type,
        JointType::Continuous
    );
}
