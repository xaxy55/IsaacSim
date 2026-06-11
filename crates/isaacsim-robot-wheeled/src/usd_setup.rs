// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Holonomic robot USD setup.
//!
//! Port of `robots/holonomic_robot_usd_setup.py`: reads mecanum-wheel
//! parameters from a stage (joints carrying `isaacmecanumwheel:angle` /
//! `isaacmecanumwheel:radius` attributes) and produces the configuration for
//! [`HolonomicController`](crate::HolonomicController). Works on any
//! [`SceneStage`] backend (in-memory or OpenUSD via `isaacsim-usd`).
//!
//! The legacy `WheeledRobot` runtime wrapper (articulation control) depends
//! on the simulation core and is not part of this port.

use isaacsim_core_math::transform::{quaternion_conjugate, quaternion_multiplication};
use isaacsim_scene::xform::{
    extract_rotation_quaternion, extract_translation, local_to_world_transform,
};
use isaacsim_scene::{descendants, SceneStage, UpAxis, Value};

use crate::holonomic::HolonomicConfig;

fn named_axis(name: &str) -> Result<[f64; 3], String> {
    match name {
        "X" => Ok([1.0, 0.0, 0.0]),
        "Y" => Ok([0.0, 1.0, 0.0]),
        "Z" => Ok([0.0, 0.0, 1.0]),
        other => Err(format!("unknown axis token {other:?}")),
    }
}

/// Mecanum robot parameters extracted from a stage, ready to drive a
/// [`HolonomicController`](crate::HolonomicController).
#[derive(Debug, Clone)]
pub struct HolonomicRobotUsdSetup {
    /// Radius of each wheel.
    pub wheel_radius: Vec<f64>,
    /// Wheel joint positions relative to the center-of-mass prim.
    pub wheel_positions: Vec<[f64; 3]>,
    /// Wheel joint orientations relative to the center-of-mass prim,
    /// as quaternions `[w, x, y, z]`.
    pub wheel_orientations: Vec<[f64; 4]>,
    /// Mecanum roller angles in degrees.
    pub mecanum_angles: Vec<f64>,
    /// DOF names of the wheel joints (prim names).
    pub wheel_dof_names: Vec<String>,
    /// Local rotation axis of the wheel joints.
    pub wheel_axis: [f64; 3],
    /// Up axis of the stage.
    pub up_axis: [f64; 3],
}

impl HolonomicRobotUsdSetup {
    /// Read mecanum robot parameters from `stage`.
    ///
    /// Searches the subtree under `robot_prim_path` for joints carrying an
    /// `isaacmecanumwheel:angle` attribute. Wheel poses are computed from
    /// each joint's `physics:localPos0`/`physics:localRot0` composed with
    /// the world transform of its `physics:body0` target, relative to
    /// `com_prim_path` (empty string: the robot prim).
    ///
    /// Errors if the robot prim does not exist or no mecanum joints are
    /// found (legacy raises `ValueError` / fails on an empty joint list).
    pub fn from_stage<S: SceneStage + ?Sized>(
        stage: &S,
        robot_prim_path: &str,
        com_prim_path: &str,
    ) -> Result<Self, String> {
        if !stage.prim_exists(robot_prim_path) {
            return Err(format!("Invalid robot prim path: {robot_prim_path}"));
        }
        let com_prim_path = if !com_prim_path.is_empty() && stage.prim_exists(com_prim_path) {
            com_prim_path
        } else {
            robot_prim_path
        };

        let mecanum_joints: Vec<String> = descendants(stage, robot_prim_path)
            .into_iter()
            .filter(|path| stage.attribute(path, "isaacmecanumwheel:angle").is_some())
            .collect();
        if mecanum_joints.is_empty() {
            return Err(format!(
                "no joints with isaacmecanumwheel:angle found under {robot_prim_path}"
            ));
        }

        let attr_f64 = |path: &str, name: &str| -> Result<f64, String> {
            match stage.attribute(path, name) {
                Some(Value::Float(v)) => Ok(v as f64),
                Some(Value::Double(v)) => Ok(v),
                Some(Value::Int(v)) => Ok(v as f64),
                Some(other) => Err(format!("{path}.{name} has non-numeric value {other:?}")),
                None => Err(format!("{path} has no {name} attribute")),
            }
        };

        let com_world = local_to_world_transform(stage, com_prim_path);
        let com_translation = extract_translation(&com_world);
        let com_rotation = extract_rotation_quaternion(&com_world);

        let mut setup = Self {
            wheel_radius: Vec::with_capacity(mecanum_joints.len()),
            wheel_positions: Vec::with_capacity(mecanum_joints.len()),
            wheel_orientations: Vec::with_capacity(mecanum_joints.len()),
            mecanum_angles: Vec::with_capacity(mecanum_joints.len()),
            wheel_dof_names: Vec::with_capacity(mecanum_joints.len()),
            wheel_axis: [1.0, 0.0, 0.0],
            up_axis: match stage.up_axis() {
                UpAxis::Y => [0.0, 1.0, 0.0],
                UpAxis::Z => [0.0, 0.0, 1.0],
            },
        };

        for joint_path in &mecanum_joints {
            setup.wheel_radius.push(attr_f64(joint_path, "isaacmecanumwheel:radius")?);
            setup.mecanum_angles.push(attr_f64(joint_path, "isaacmecanumwheel:angle")?);
            setup
                .wheel_dof_names
                .push(joint_path.rsplit('/').next().unwrap_or(joint_path).to_string());

            // joint world pose = chassis world transform * joint local frame 0
            let chassis = stage
                .relationship_targets(joint_path, "physics:body0")
                .and_then(|targets| targets.into_iter().next())
                .ok_or_else(|| format!("{joint_path} has no physics:body0 target"))?;
            let chassis_world = local_to_world_transform(stage, &chassis);

            let local_pos = stage
                .attribute(joint_path, "physics:localPos0")
                .and_then(|v| v.as_vec3d())
                .unwrap_or([0.0, 0.0, 0.0]);
            let local_rot = stage
                .attribute(joint_path, "physics:localRot0")
                .and_then(|v| v.as_quatd())
                .unwrap_or([1.0, 0.0, 0.0, 0.0]);

            let chassis_rotation = extract_rotation_quaternion(&chassis_world);
            let joint_world_rotation = quaternion_multiplication(chassis_rotation, local_rot);
            // chassis_world * local_pos (column convention)
            let joint_world_position = [
                chassis_world[0][0] * local_pos[0]
                    + chassis_world[0][1] * local_pos[1]
                    + chassis_world[0][2] * local_pos[2]
                    + chassis_world[0][3],
                chassis_world[1][0] * local_pos[0]
                    + chassis_world[1][1] * local_pos[1]
                    + chassis_world[1][2] * local_pos[2]
                    + chassis_world[1][3],
                chassis_world[2][0] * local_pos[0]
                    + chassis_world[2][1] * local_pos[1]
                    + chassis_world[2][2] * local_pos[2]
                    + chassis_world[2][3],
            ];

            setup.wheel_positions.push([
                joint_world_position[0] - com_translation[0],
                joint_world_position[1] - com_translation[1],
                joint_world_position[2] - com_translation[2],
            ]);
            // Rotation relative to the com frame: com_rotation^-1 * joint_rotation
            setup.wheel_orientations.push(quaternion_multiplication(
                quaternion_conjugate(com_rotation),
                joint_world_rotation,
            ));
        }

        // Wheel axis comes from the last mecanum joint (legacy behavior)
        let last = mecanum_joints.last().expect("non-empty");
        setup.wheel_axis = match stage.attribute(last, "physics:axis") {
            Some(Value::Token(axis)) => named_axis(&axis)?,
            _ => [1.0, 0.0, 0.0],
        };

        Ok(setup)
    }

    /// Parameters for [`HolonomicController`](crate::HolonomicController),
    /// matching the legacy `get_holonomic_controller_params` tuple.
    pub fn holonomic_controller_config(&self) -> HolonomicConfig {
        HolonomicConfig {
            wheel_radius: self.wheel_radius.clone(),
            wheel_positions: self.wheel_positions.clone(),
            wheel_orientations: self.wheel_orientations.clone(),
            mecanum_angles: self.mecanum_angles.clone(),
            wheel_axis: self.wheel_axis,
            up_axis: self.up_axis,
            ..Default::default()
        }
    }

    /// DOF names for the mecanum wheel joints
    /// (legacy `get_articulation_controller_params`).
    pub fn articulation_controller_params(&self) -> &[String] {
        &self.wheel_dof_names
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::HolonomicController;
    use isaacsim_scene::Stage;

    /// Author the kiwi-drive fixture from the legacy holonomic controller
    /// test as a physics-jointed robot on `stage`.
    pub(crate) fn author_kaya_like_robot(stage: &mut dyn SceneStage) {
        stage.define_prim("/Robot", "Xform").unwrap();
        stage.define_prim("/Robot/Chassis", "Xform").unwrap();
        let wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
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
                .set_attribute(
                    &path,
                    "physics:localPos0",
                    Value::Vec3f([
                        wheel_positions[i][0] as f32,
                        wheel_positions[i][1] as f32,
                        wheel_positions[i][2] as f32,
                    ]),
                )
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

    /// End-to-end port of the legacy holonomic flow: extract parameters from
    /// the stage, build the controller, and reproduce the legacy
    /// `test_holonomic_drive` reference values.
    #[test]
    fn test_setup_from_stage_drives_controller() {
        let mut stage = Stage::new();
        author_kaya_like_robot(&mut stage);

        let setup = HolonomicRobotUsdSetup::from_stage(&stage, "/Robot", "").unwrap();
        assert_eq!(setup.wheel_dof_names, ["wheel_joint_0", "wheel_joint_1", "wheel_joint_2"]);
        assert_eq!(setup.wheel_axis, [1.0, 0.0, 0.0]);
        assert_eq!(setup.up_axis, [0.0, 0.0, 1.0]);
        assert_eq!(setup.mecanum_angles, [90.0, 90.0, 90.0]);

        let mut controller = HolonomicController::new(setup.holonomic_controller_config()).unwrap();
        let actions = controller.forward([1.0, 1.0, 0.1]);
        assert!((actions[0] - -25.105).abs() < 0.01, "got {actions:?}");
        assert!((actions[1] - 14.3182).abs() < 0.01, "got {actions:?}");
        assert!((actions[2] - -14.5417).abs() < 0.01, "got {actions:?}");
    }

    /// Wheel poses are taken relative to the center-of-mass prim, and a
    /// chassis offset shifts the extracted positions.
    #[test]
    fn test_chassis_offset_is_relative_to_com() {
        let mut stage = Stage::new();
        author_kaya_like_robot(&mut stage);
        stage
            .set_attribute("/Robot/Chassis", "xformOp:translate", Value::Vec3d([1.0, 2.0, 0.0]))
            .unwrap();

        // com prim = robot root at the origin: positions shift by the offset
        let setup = HolonomicRobotUsdSetup::from_stage(&stage, "/Robot", "").unwrap();
        assert!((setup.wheel_positions[0][0] - (1.0 - 0.0980432)).abs() < 1e-6);
        assert!((setup.wheel_positions[0][1] - (2.0 + 0.000636773)).abs() < 1e-6);

        // com prim = chassis itself: positions are local again
        let setup =
            HolonomicRobotUsdSetup::from_stage(&stage, "/Robot", "/Robot/Chassis").unwrap();
        assert!((setup.wheel_positions[0][0] - -0.0980432).abs() < 1e-6);
    }

    #[test]
    fn test_missing_robot_prim_fails() {
        let stage = Stage::new();
        let err = HolonomicRobotUsdSetup::from_stage(&stage, "/Missing", "").unwrap_err();
        assert!(err.contains("Invalid robot prim path"), "{err}");
    }
}
