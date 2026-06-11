// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! End-to-end: parse a legacy URDF fixture, merge fixed joints, convert to a
//! stage, export `.usda`, and round-trip through the subset reader.
//!
//! The exported file is also written to the tmp dir so it can be validated
//! against real USD (`pxr.Usd` + `UsdPhysics`) out of band.

use isaacsim_asset_urdf::{convert_urdf_to_stage, merge_fixed_joints, parse_urdf_file};
use isaacsim_scene::{Stage, Value};

fn fixture(name: &str) -> String {
    format!(
        "{}/../../legacy/source/extensions/isaacsim.asset.importer.urdf/data/urdf/tests/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

#[test]
fn test_convert_test_limits_fixture() {
    let mut robot = parse_urdf_file(fixture("test_limits.urdf")).unwrap();
    merge_fixed_joints(&mut robot);
    let mut stage = Stage::new();
    convert_urdf_to_stage(&robot, &mut stage, "/test_limits").unwrap();

    // The fixed root_to_base joint was merged away; the chain is rooted at
    // root_link with continuous + revolute joints below it
    assert!(stage.prim_exists("/test_limits/root_link"));
    assert!(!stage.prim_exists("/test_limits/base_link"));
    assert!(stage.has_api_schema("/test_limits", "PhysicsArticulationRootAPI"));
    assert!(stage.has_api_schema("/test_limits/link_1", "PhysicsRigidBodyAPI"));
    assert!(stage.has_api_schema("/test_limits/link_1", "PhysicsMassAPI"));

    // Continuous joint: revolute prim without limits
    let base_joint = "/test_limits/joints/base_joint";
    assert_eq!(
        stage.composed_type_name(base_joint).unwrap(),
        "PhysicsRevoluteJoint"
    );
    assert_eq!(stage.resolve_attribute(base_joint, "physics:lowerLimit"), None);

    // Revolute joint: limits converted to degrees
    let elbow = "/test_limits/joints/elbow_joint";
    let Some(Value::Float(lower)) = stage.resolve_attribute(elbow, "physics:lowerLimit") else {
        panic!("missing elbow lower limit");
    };
    assert!((lower - (-0.6f32).to_degrees()).abs() < 1e-3);

    // Link world poses accumulate down the chain: link_2 sits at
    // base(0,0,0) + base_joint(0,0,0.45) + elbow(0,0,0.4)
    let assert_translate = |stage: &Stage| {
        let Some(Value::Vec3d(t)) =
            stage.resolve_attribute("/test_limits/link_2", "xformOp:translate")
        else {
            panic!("missing link_2 translate");
        };
        assert!(t[0].abs() < 1e-12 && t[1].abs() < 1e-12 && (t[2] - 0.85).abs() < 1e-12, "{t:?}");
    };
    assert_translate(&stage);

    // Round-trip through the usda subset reader preserves the physics data
    let parsed = Stage::from_usda(&stage.to_usda()).unwrap();
    assert!(parsed.has_api_schema("/test_limits/link_1", "PhysicsRigidBodyAPI"));
    assert_eq!(
        parsed.relationship_targets(elbow, "physics:body1"),
        Some(vec!["/test_limits/link_2".to_string()])
    );
    assert_translate(&parsed);

    // Persist for out-of-band validation with real USD
    stage
        .save_usda(std::env::temp_dir().join("isaacsim_urdf_converted.usda"))
        .unwrap();
}
