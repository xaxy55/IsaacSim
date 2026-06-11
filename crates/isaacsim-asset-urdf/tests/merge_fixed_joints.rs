// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Port of legacy `python/tests/test_urdf_utils.py` (merge_fixed_joints),
//! asserting on the parsed model instead of re-parsed XML output.

use isaacsim_asset_urdf::{merge_fixed_joints, parse_urdf, JointType, Robot};

fn merged(urdf: &str) -> Robot {
    let mut robot = parse_urdf(urdf).unwrap();
    merge_fixed_joints(&mut robot);
    robot
}

fn link_names(robot: &Robot) -> Vec<&str> {
    robot.links.iter().map(|l| l.name.as_str()).collect()
}

fn joint_names(robot: &Robot) -> Vec<&str> {
    robot.joints.iter().map(|j| j.name.as_str()).collect()
}

fn assert_close(actual: [f64; 3], expected: [f64; 3]) {
    for (a, e) in actual.iter().zip(&expected) {
        assert!((a - e).abs() < 1e-9, "expected {expected:?}, got {actual:?}");
    }
}

// -- no-op cases -------------------------------------------------------------

/// Port of legacy `test_no_fixed_joints_unchanged`.
#[test]
fn test_no_fixed_joints_unchanged() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="link1">
            <visual><geometry><box size="1 1 1"/></geometry></visual>
          </link>
          <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>"#,
    );
    assert_eq!(link_names(&robot), ["base", "link1"]);
    assert_eq!(joint_names(&robot), ["j1"]);
}

/// Port of legacy `test_empty_robot`.
#[test]
fn test_empty_robot() {
    let robot = merged(r#"<robot name="empty"></robot>"#);
    assert!(robot.links.is_empty());
    assert!(robot.joints.is_empty());
}

// -- single fixed joint ------------------------------------------------------

/// Port of legacy `test_single_fixed_joint_removes_child_link`.
#[test]
fn test_single_fixed_joint_removes_child_link() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="child">
            <visual><geometry><box size="1 1 1"/></geometry></visual>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    assert_eq!(link_names(&robot), ["base"]);
    assert!(robot.joints.is_empty());
}

/// Port of legacy `test_visual_transferred_to_parent` and
/// `test_collision_transferred_to_parent`.
#[test]
fn test_visuals_and_collisions_transferred_to_parent() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base">
            <visual><geometry><sphere radius="0.1"/></geometry></visual>
          </link>
          <link name="child">
            <visual><geometry><box size="1 1 1"/></geometry></visual>
            <collision><geometry><box size="1 1 1"/></geometry></collision>
            <collision><geometry><sphere radius="0.5"/></geometry></collision>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let base = robot.link("base").unwrap();
    assert_eq!(base.visuals.len(), 2);
    assert_eq!(base.collisions.len(), 2);
}

// -- transform composition ---------------------------------------------------

/// Port of legacy `test_visual_origin_composed`.
#[test]
fn test_visual_origin_composed() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="child">
            <visual>
              <origin xyz="0.5 0 0" rpy="0 0 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </visual>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let base = robot.link("base").unwrap();
    assert_close(base.visuals[0].origin.xyz, [1.5, 0.0, 0.0]);
}

/// Port of legacy `test_collision_origin_composed`.
#[test]
fn test_collision_origin_composed() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="child">
            <collision>
              <origin xyz="0 0.2 0" rpy="0 0 0"/>
              <geometry><sphere radius="0.1"/></geometry>
            </collision>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="0 0 3" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let base = robot.link("base").unwrap();
    assert_close(base.collisions[0].origin.xyz, [0.0, 0.2, 3.0]);
}

/// Port of legacy `test_rotation_composition` (90-deg yaw rotates the child
/// translation).
#[test]
fn test_rotation_composition() {
    let yaw = std::f64::consts::FRAC_PI_2;
    let robot = merged(&format!(
        r#"<robot name="test">
          <link name="base"/>
          <link name="child">
            <visual>
              <origin xyz="1 0 0" rpy="0 0 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </visual>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="0 0 0" rpy="0 0 {yaw}"/>
          </joint>
        </robot>"#
    ));
    let base = robot.link("base").unwrap();
    assert_close(base.visuals[0].origin.xyz, [0.0, 1.0, 0.0]);
}

// -- joint re-parenting ------------------------------------------------------

/// Port of legacy `test_revolute_reparented_to_parent` and
/// `test_reparented_joint_origin_composed`.
#[test]
fn test_revolute_reparented_with_composed_origin() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="adapter"/>
          <link name="end"/>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="adapter"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
          <joint name="revolute_j" type="revolute">
            <parent link="adapter"/>
            <child link="end"/>
            <origin xyz="2 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>"#,
    );
    assert!(!link_names(&robot).contains(&"adapter"));
    assert_eq!(robot.joints.len(), 1);
    let joint = &robot.joints[0];
    assert_eq!(joint.name, "revolute_j");
    assert_eq!(joint.parent, "base");
    assert_eq!(joint.child, "end");
    assert_close(joint.origin.xyz, [3.0, 0.0, 0.0]);
}

/// Port of legacy `test_multiple_downstream_joints_reparented`.
#[test]
fn test_multiple_downstream_joints_reparented() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="adapter"/>
          <link name="arm"/>
          <link name="hand"/>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="adapter"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
          <joint name="arm_j" type="revolute">
            <parent link="adapter"/>
            <child link="arm"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="hand_j" type="revolute">
            <parent link="adapter"/>
            <child link="hand"/>
            <origin xyz="0 1 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>"#,
    );
    assert!(!link_names(&robot).contains(&"adapter"));
    for joint in &robot.joints {
        assert_eq!(joint.parent, "base");
    }
}

// -- chains of fixed joints --------------------------------------------------

/// Port of legacy `test_chain_of_two_fixed_joints` and
/// `test_chain_transform_accumulation`.
#[test]
fn test_chain_of_fixed_joints_accumulates_transforms() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="mid">
            <visual><geometry><box size="1 1 1"/></geometry></visual>
          </link>
          <link name="tip">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><sphere radius="0.1"/></geometry>
            </visual>
          </link>
          <joint name="j1" type="fixed">
            <parent link="base"/>
            <child link="mid"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
          <joint name="j2" type="fixed">
            <parent link="mid"/>
            <child link="tip"/>
            <origin xyz="0 2 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    assert_eq!(link_names(&robot), ["base"]);
    assert!(robot.joints.is_empty());
    let base = robot.link("base").unwrap();
    assert_eq!(base.visuals.len(), 2);
    // The tip visual accumulated both fixed transforms
    assert_close(base.visuals[1].origin.xyz, [1.0, 2.0, 0.0]);
}

/// Port of legacy `test_chain_with_trailing_revolute`.
#[test]
fn test_chain_with_trailing_revolute() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="mid"/>
          <link name="adapter"/>
          <link name="end"/>
          <joint name="j1" type="fixed">
            <parent link="base"/>
            <child link="mid"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
          <joint name="j2" type="fixed">
            <parent link="mid"/>
            <child link="adapter"/>
            <origin xyz="0 1 0" rpy="0 0 0"/>
          </joint>
          <joint name="j3" type="revolute">
            <parent link="adapter"/>
            <child link="end"/>
            <origin xyz="0 0 1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>"#,
    );
    let mut names = link_names(&robot);
    names.sort_unstable();
    assert_eq!(names, ["base", "end"]);
    assert_eq!(robot.joints.len(), 1);
    let joint = &robot.joints[0];
    assert_eq!(joint.name, "j3");
    assert_eq!(joint.parent, "base");
    assert_close(joint.origin.xyz, [1.0, 1.0, 1.0]);
}

// -- inertial merging --------------------------------------------------------

/// Port of legacy `test_mass_is_summed`.
#[test]
fn test_mass_is_summed() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base">
            <inertial>
              <mass value="2.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <link name="child">
            <inertial>
              <mass value="3.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.2" iyy="0.2" izz="0.2" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let inertial = robot.link("base").unwrap().inertial.unwrap();
    assert!((inertial.mass - 5.0).abs() < 1e-12);
}

/// Port of legacy `test_com_weighted_average` and
/// `test_inertia_parallel_axis_theorem`.
#[test]
fn test_com_weighted_average_and_parallel_axis() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base">
            <inertial>
              <mass value="1.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <link name="child">
            <inertial>
              <mass value="1.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="2 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let inertial = robot.link("base").unwrap().inertial.unwrap();
    assert_close(inertial.origin.xyz, [1.0, 0.0, 0.0]);
    // Two point masses of 1 kg at distance 1 m from the combined CoM
    assert!((inertial.inertia.ixx).abs() < 1e-6);
    assert!((inertial.inertia.iyy - 2.0).abs() < 1e-6);
    assert!((inertial.inertia.izz - 2.0).abs() < 1e-6);
    assert!((inertial.inertia.ixy).abs() < 1e-6);
    assert!((inertial.inertia.ixz).abs() < 1e-6);
    assert!((inertial.inertia.iyz).abs() < 1e-6);
}

/// Port of legacy `test_child_without_inertial`.
#[test]
fn test_child_without_inertial() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base">
            <inertial>
              <mass value="5.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <link name="child"/>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let inertial = robot.link("base").unwrap().inertial.unwrap();
    assert!((inertial.mass - 5.0).abs() < 1e-12);
}

/// Port of legacy `test_parent_without_inertial_inherits_child`.
#[test]
fn test_parent_without_inertial_inherits_child() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="child">
            <inertial>
              <mass value="3.0"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
          <joint name="fixed_j" type="fixed">
            <parent link="base"/>
            <child link="child"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let inertial = robot.link("base").unwrap().inertial.unwrap();
    assert!((inertial.mass - 3.0).abs() < 1e-12);
    assert_close(inertial.origin.xyz, [1.0, 0.0, 0.0]);
}

// -- mixed topology ----------------------------------------------------------

/// Port of legacy `test_fixed_between_two_revolutes`.
#[test]
fn test_fixed_between_two_revolutes() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="A"/>
          <link name="B">
            <visual><geometry><box size="1 1 1"/></geometry></visual>
          </link>
          <link name="C"/>
          <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="A"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="j_fixed" type="fixed">
            <parent link="A"/>
            <child link="B"/>
            <origin xyz="0 0 0.5" rpy="0 0 0"/>
          </joint>
          <joint name="j2" type="revolute">
            <parent link="B"/>
            <child link="C"/>
            <origin xyz="0 0 1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>"#,
    );
    let mut names = link_names(&robot);
    names.sort_unstable();
    assert_eq!(names, ["A", "C", "base"]);
    assert_eq!(joint_names(&robot), ["j1", "j2"]);
    let j2 = robot.joint("j2").unwrap();
    assert_eq!(j2.parent, "A");
    assert_close(j2.origin.xyz, [0.0, 0.0, 1.5]);
    assert_eq!(robot.link("A").unwrap().visuals.len(), 1);
}

/// Port of legacy `test_only_fixed_joints_removed`.
#[test]
fn test_only_fixed_joints_removed() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <link name="A"/>
          <link name="B"/>
          <link name="C"/>
          <joint name="j_rev" type="revolute">
            <parent link="base"/>
            <child link="A"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="j_cont" type="continuous">
            <parent link="A"/>
            <child link="B"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="j_fixed" type="fixed">
            <parent link="B"/>
            <child link="C"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>"#,
    );
    let types: Vec<JointType> = robot.joints.iter().map(|j| j.joint_type).collect();
    assert!(!types.contains(&JointType::Fixed));
    assert!(types.contains(&JointType::Revolute));
    assert!(types.contains(&JointType::Continuous));
    assert!(!link_names(&robot).contains(&"C"));
}

/// Self-loop fixed joints are dropped without removing the link
/// (legacy logs an error and drops the joint).
#[test]
fn test_self_loop_fixed_joint_dropped() {
    let robot = merged(
        r#"<robot name="test">
          <link name="base"/>
          <joint name="loop" type="fixed">
            <parent link="base"/>
            <child link="base"/>
          </joint>
        </robot>"#,
    );
    assert_eq!(link_names(&robot), ["base"]);
    assert!(robot.joints.is_empty());
}
