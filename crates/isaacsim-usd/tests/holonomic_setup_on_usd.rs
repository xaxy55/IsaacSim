// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! The ported `HolonomicRobotUsdSetup` reading a real OpenUSD stage:
//! author a kiwi-drive robot with physics joints, extract the mecanum
//! parameters through the FFI backend, and reproduce the legacy
//! `test_holonomic_drive` reference values.

#![cfg(feature = "openusd")]

use isaacsim_robot_wheeled::{HolonomicController, HolonomicRobotUsdSetup};
use isaacsim_scene::{SceneStage, UpAxis, Value};
use isaacsim_usd::UsdStage;

fn author_kaya_like_robot(stage: &mut dyn SceneStage) {
    stage.define_prim("/Robot", "Xform").unwrap();
    stage.define_prim("/Robot/Chassis", "Xform").unwrap();
    let wheel_positions = [
        [-0.0980432f32, 0.000636773, -0.050501],
        [0.0493475, -0.084525, -0.050501],
        [0.0495291, 0.0856937, -0.050501],
    ];
    let wheel_orientations: [[f32; 4]; 3] = [
        [0.0, 0.0, 0.0, 1.0],
        [0.866, 0.0, 0.0, -0.5],
        [0.866, 0.0, 0.0, 0.5],
    ];
    for i in 0..3 {
        let path = format!("/Robot/Joints/wheel_joint_{i}");
        stage.define_prim(&path, "PhysicsRevoluteJoint").unwrap();
        stage
            .set_relationship_targets(&path, "physics:body0", &["/Robot/Chassis".to_string()])
            .unwrap();
        stage
            .set_attribute(&path, "physics:localPos0", Value::Vec3f(wheel_positions[i]))
            .unwrap();
        stage
            .set_attribute(&path, "physics:localRot0", Value::Quatf(wheel_orientations[i]))
            .unwrap();
        stage
            .set_attribute(&path, "physics:axis", Value::Token("X".to_string()))
            .unwrap();
        stage
            .set_attribute(&path, "isaacmecanumwheel:angle", Value::Float(90.0))
            .unwrap();
        stage
            .set_attribute(&path, "isaacmecanumwheel:radius", Value::Float(0.04))
            .unwrap();
    }
}

#[test]
fn test_holonomic_setup_on_real_usd() {
    let mut stage = UsdStage::create_in_memory().unwrap();
    stage.set_up_axis(UpAxis::Z);
    author_kaya_like_robot(&mut stage);

    let setup = HolonomicRobotUsdSetup::from_stage(&stage, "/Robot", "").unwrap();
    assert_eq!(
        setup.wheel_dof_names,
        ["wheel_joint_0", "wheel_joint_1", "wheel_joint_2"]
    );
    assert_eq!(setup.wheel_axis, [1.0, 0.0, 0.0]);
    assert_eq!(setup.up_axis, [0.0, 0.0, 1.0]);

    let mut controller = HolonomicController::new(setup.holonomic_controller_config()).unwrap();
    let actions = controller.forward([1.0, 1.0, 0.1]);
    assert!((actions[0] - -25.105).abs() < 0.01, "got {actions:?}");
    assert!((actions[1] - 14.3182).abs() < 0.01, "got {actions:?}");
    assert!((actions[2] - -14.5417).abs() < 0.01, "got {actions:?}");
}

#[test]
fn test_relationships_round_trip_through_real_usd() {
    let mut stage = UsdStage::create_in_memory().unwrap();
    stage.define_prim("/World/A", "Xform").unwrap();
    stage.define_prim("/World/B", "Xform").unwrap();
    stage
        .set_relationship_targets(
            "/World/A",
            "physics:filteredGroups",
            &["/World/B".to_string(), "/World/A".to_string()],
        )
        .unwrap();
    assert_eq!(
        stage.relationship_targets("/World/A", "physics:filteredGroups"),
        Some(vec!["/World/B".to_string(), "/World/A".to_string()])
    );
    assert_eq!(stage.relationship_targets("/World/A", "missing"), None);
    assert_eq!(stage.children("/World"), vec!["/World/A", "/World/B"]);
}
