// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! URDF → stage conversion (rigid-body subset).
//!
//! Rust-native replacement for the rigid-body core of the external
//! `urdf_usd_converter`: authors a parsed [`Robot`] onto any
//! [`SceneStage`] using `UsdGeom` + `UsdPhysics` conventions, so the result
//! opens in real USD/Isaac Sim (via `isaacsim-usd` or `.usda` export).
//!
//! Layout (one Xform per link at its zero-configuration world pose):
//!
//! ```text
//! /<root>                    Xform + PhysicsArticulationRootAPI
//!   /<link>                  Xform + PhysicsRigidBodyAPI [+ PhysicsMassAPI]
//!     /visuals/<name>        Cube | Sphere | Cylinder | Capsule | Mesh
//!     /collisions/<name>     same, with purpose = "guide" + PhysicsCollisionAPI
//!   /joints/<joint>          PhysicsRevoluteJoint | PhysicsPrismaticJoint |
//!                            PhysicsFixedJoint
//! ```
//!
//! Out of scope (downstream of the parsing/authoring boundary): mesh file
//! tessellation (mesh prims carry the source filename), drives/actuation,
//! and `floating`/`planar` joints (skipped — the child body is simply free).

use isaacsim_core_math::linalg::sym3_eigen;
use isaacsim_core_math::transform::{
    euler_angles_to_quaternion, quaternion_multiplication, rotation_matrix_to_quaternion,
};
use isaacsim_scene::{SceneStage, Value};

use crate::model::{Geometry, Inertial, Joint, JointType, Origin, Robot};
use crate::transform::{compose_origins, origin_to_matrix};

/// Sanitize a URDF name into a USD prim name.
fn prim_name(name: &str) -> String {
    let mut out: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' })
        .collect();
    if out.is_empty() || out.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    out
}

/// Shortest-arc quaternion rotating the +X axis onto `axis`
/// (identity for zero-length axes).
fn x_axis_alignment(axis: [f64; 3]) -> [f64; 4] {
    let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if norm == 0.0 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let a = [axis[0] / norm, axis[1] / norm, axis[2] / norm];
    let dot = a[0]; // x_hat . a
    if dot < -1.0 + 1e-12 {
        // Opposite direction: 180 degrees about Z
        return [0.0, 0.0, 0.0, 1.0];
    }
    // q = normalize(1 + x_hat . a, x_hat x a)
    let w = 1.0 + dot;
    let xyz = [0.0, -a[2], a[1]]; // x_hat x a = (0, -az, ay)
    let n = (w * w + xyz[1] * xyz[1] + xyz[2] * xyz[2]).sqrt();
    [w / n, 0.0, xyz[1] / n, xyz[2] / n]
}

fn origin_quaternion(origin: &Origin) -> [f64; 4] {
    // URDF rpy is fixed-axis (extrinsic XYZ): Rz * Ry * Rx
    euler_angles_to_quaternion(origin.rpy, false, true)
}

fn quatf(q: [f64; 4]) -> Value {
    Value::Quatf([q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32])
}

fn set_xform<S: SceneStage + ?Sized>(
    stage: &mut S,
    path: &str,
    translate: [f64; 3],
    orient: [f64; 4],
    scale: [f64; 3],
) -> Result<(), String> {
    stage.set_attribute(path, "xformOp:translate", Value::Vec3d(translate))?;
    stage.set_attribute(path, "xformOp:orient", Value::Quatd(orient))?;
    stage.set_attribute(path, "xformOp:scale", Value::Vec3d(scale))?;
    stage.set_attribute(
        path,
        "xformOpOrder",
        Value::TokenArray(vec![
            "xformOp:translate".to_string(),
            "xformOp:orient".to_string(),
            "xformOp:scale".to_string(),
        ]),
    )
}

/// Author `robot` onto `stage` under `root_path` and return the root path.
///
/// Link world poses are computed for the zero joint configuration by
/// walking the kinematic tree; links not reachable from a root link are
/// placed at the identity pose.
pub fn convert_urdf_to_stage<S: SceneStage + ?Sized>(
    robot: &Robot,
    stage: &mut S,
    root_path: &str,
) -> Result<String, String> {
    stage.define_prim(root_path, "Xform")?;
    stage.apply_api_schema(root_path, "PhysicsArticulationRootAPI")?;

    // Zero-configuration world pose per link: child = parent * joint.origin
    let mut poses: std::collections::BTreeMap<&str, Origin> = robot
        .links
        .iter()
        .map(|l| (l.name.as_str(), Origin::default()))
        .collect();
    let mut resolved: std::collections::BTreeSet<&str> = robot
        .links
        .iter()
        .filter(|l| !robot.joints.iter().any(|j| j.child == l.name))
        .map(|l| l.name.as_str())
        .collect();
    loop {
        let mut progressed = false;
        for joint in &robot.joints {
            if resolved.contains(joint.parent.as_str()) && !resolved.contains(joint.child.as_str())
            {
                let parent_pose = poses[joint.parent.as_str()];
                let pose = compose_origins(&origin_to_matrix(&parent_pose), &joint.origin);
                poses.insert(&joint.child, pose);
                resolved.insert(&joint.child);
                progressed = true;
            }
        }
        if !progressed {
            break;
        }
    }

    // Links
    let mut link_paths: std::collections::BTreeMap<&str, String> = Default::default();
    for link in &robot.links {
        let path = format!("{root_path}/{}", prim_name(&link.name));
        stage.define_prim(&path, "Xform")?;
        stage.apply_api_schema(&path, "PhysicsRigidBodyAPI")?;
        let pose = poses[link.name.as_str()];
        set_xform(stage, &path, pose.xyz, origin_quaternion(&pose), [1.0, 1.0, 1.0])?;

        if let Some(inertial) = &link.inertial {
            author_mass(stage, &path, inertial)?;
        }
        for (i, visual) in link.visuals.iter().enumerate() {
            let name = visual.name.clone().unwrap_or_else(|| format!("visual_{i}"));
            author_geometry(stage, &format!("{path}/visuals/{}", prim_name(&name)), &visual.origin, &visual.geometry, false)?;
        }
        for (i, collision) in link.collisions.iter().enumerate() {
            let name = collision.name.clone().unwrap_or_else(|| format!("collision_{i}"));
            author_geometry(stage, &format!("{path}/collisions/{}", prim_name(&name)), &collision.origin, &collision.geometry, true)?;
        }
        link_paths.insert(&link.name, path);
    }

    // Joints
    for joint in &robot.joints {
        author_joint(stage, root_path, joint, &link_paths)?;
    }

    Ok(root_path.to_string())
}

/// Author UsdPhysics mass attributes: mass, center of mass, and the inertia
/// tensor expressed as diagonal inertia + principal axes (eigendecomposition
/// of the symmetric tensor).
fn author_mass<S: SceneStage + ?Sized>(
    stage: &mut S,
    path: &str,
    inertial: &Inertial,
) -> Result<(), String> {
    stage.apply_api_schema(path, "PhysicsMassAPI")?;
    stage.set_attribute(path, "physics:mass", Value::Float(inertial.mass as f32))?;
    stage.set_attribute(
        path,
        "physics:centerOfMass",
        Value::Vec3f([
            inertial.origin.xyz[0] as f32,
            inertial.origin.xyz[1] as f32,
            inertial.origin.xyz[2] as f32,
        ]),
    )?;
    let i = &inertial.inertia;
    let tensor = [
        [i.ixx, i.ixy, i.ixz],
        [i.ixy, i.iyy, i.iyz],
        [i.ixz, i.iyz, i.izz],
    ];
    let (eigenvalues, eigenvectors) = sym3_eigen(&tensor);
    // Columns of the principal-axes rotation are the eigenvectors
    let principal = [
        [eigenvectors[0][0], eigenvectors[1][0], eigenvectors[2][0]],
        [eigenvectors[0][1], eigenvectors[1][1], eigenvectors[2][1]],
        [eigenvectors[0][2], eigenvectors[1][2], eigenvectors[2][2]],
    ];
    stage.set_attribute(
        path,
        "physics:diagonalInertia",
        Value::Vec3f([
            eigenvalues[0] as f32,
            eigenvalues[1] as f32,
            eigenvalues[2] as f32,
        ]),
    )?;
    // The inertial origin rpy rotates the tensor frame relative to the link
    let q = quaternion_multiplication(
        origin_quaternion(&inertial.origin),
        rotation_matrix_to_quaternion(&principal),
    );
    stage.set_attribute(path, "physics:principalAxes", quatf(q))
}

fn author_geometry<S: SceneStage + ?Sized>(
    stage: &mut S,
    path: &str,
    origin: &Origin,
    geometry: &Geometry,
    is_collision: bool,
) -> Result<(), String> {
    let (type_name, scale) = match geometry {
        // A unit Cube scaled to the box dimensions
        Geometry::Box { size } => ("Cube", *size),
        Geometry::Sphere { .. } => ("Sphere", [1.0, 1.0, 1.0]),
        Geometry::Cylinder { .. } => ("Cylinder", [1.0, 1.0, 1.0]),
        Geometry::Capsule { .. } => ("Capsule", [1.0, 1.0, 1.0]),
        Geometry::Mesh { scale, .. } => ("Mesh", *scale),
    };
    stage.define_prim(path, type_name)?;
    set_xform(stage, path, origin.xyz, origin_quaternion(origin), scale)?;
    match geometry {
        Geometry::Box { .. } => {
            stage.set_attribute(path, "size", Value::Double(1.0))?;
        }
        Geometry::Sphere { radius } => {
            stage.set_attribute(path, "radius", Value::Double(*radius))?;
        }
        Geometry::Cylinder { radius, length } | Geometry::Capsule { radius, length } => {
            stage.set_attribute(path, "radius", Value::Double(*radius))?;
            stage.set_attribute(path, "height", Value::Double(*length))?;
            // URDF cylinders/capsules are Z-aligned, matching the USD default
            stage.set_attribute(path, "axis", Value::Token("Z".to_string()))?;
        }
        Geometry::Mesh { filename, .. } => {
            // Mesh tessellation is downstream; record the source asset
            stage.set_attribute(path, "isaac:urdf:meshFile", Value::String(filename.clone()))?;
        }
    }
    if is_collision {
        stage.apply_api_schema(path, "PhysicsCollisionAPI")?;
        stage.set_attribute(path, "purpose", Value::Token("guide".to_string()))?;
    }
    Ok(())
}

fn author_joint<S: SceneStage + ?Sized>(
    stage: &mut S,
    root_path: &str,
    joint: &Joint,
    link_paths: &std::collections::BTreeMap<&str, String>,
) -> Result<(), String> {
    let type_name = match joint.joint_type {
        JointType::Fixed => "PhysicsFixedJoint",
        JointType::Revolute | JointType::Continuous => "PhysicsRevoluteJoint",
        JointType::Prismatic => "PhysicsPrismaticJoint",
        // No standard UsdPhysics equivalent: the child body is simply free
        JointType::Floating | JointType::Planar => return Ok(()),
    };
    let (Some(parent_path), Some(child_path)) = (
        link_paths.get(joint.parent.as_str()),
        link_paths.get(joint.child.as_str()),
    ) else {
        return Err(format!(
            "joint {:?} references missing link(s) {:?} -> {:?}",
            joint.name, joint.parent, joint.child
        ));
    };

    let path = format!("{root_path}/joints/{}", prim_name(&joint.name));
    stage.define_prim(&path, type_name)?;
    stage.set_relationship_targets(&path, "physics:body0", std::slice::from_ref(parent_path))?;
    stage.set_relationship_targets(&path, "physics:body1", std::slice::from_ref(child_path))?;

    // USD joint axes are canonical tokens; fold the URDF axis vector into
    // the joint local rotations by aligning +X with it.
    let align = x_axis_alignment(joint.axis);
    let rot0 = quaternion_multiplication(origin_quaternion(&joint.origin), align);
    stage.set_attribute(
        &path,
        "physics:localPos0",
        Value::Vec3f([
            joint.origin.xyz[0] as f32,
            joint.origin.xyz[1] as f32,
            joint.origin.xyz[2] as f32,
        ]),
    )?;
    stage.set_attribute(&path, "physics:localRot0", quatf(rot0))?;
    stage.set_attribute(&path, "physics:localPos1", Value::Vec3f([0.0, 0.0, 0.0]))?;
    stage.set_attribute(&path, "physics:localRot1", quatf(align))?;

    if matches!(
        joint.joint_type,
        JointType::Revolute | JointType::Continuous | JointType::Prismatic
    ) {
        stage.set_attribute(&path, "physics:axis", Value::Token("X".to_string()))?;
    }
    // Continuous joints are unlimited; UsdPhysics revolute limits are in
    // degrees, prismatic limits in stage units
    if joint.joint_type != JointType::Continuous {
        if let Some(limit) = &joint.limit {
            let (lower, upper) = if joint.joint_type == JointType::Revolute {
                (limit.lower.to_degrees(), limit.upper.to_degrees())
            } else {
                (limit.lower, limit.upper)
            };
            stage.set_attribute(&path, "physics:lowerLimit", Value::Float(lower as f32))?;
            stage.set_attribute(&path, "physics:upperLimit", Value::Float(upper as f32))?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::parse_urdf;
    use isaacsim_scene::Stage;

    const SIMPLE_ARM: &str = r#"<robot name="arm">
      <link name="base">
        <inertial>
          <mass value="2.0"/>
          <origin xyz="0 0 0.1"/>
          <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
          <geometry><box size="0.2 0.3 0.4"/></geometry>
        </visual>
        <collision>
          <geometry><sphere radius="0.25"/></geometry>
        </collision>
      </link>
      <link name="upper-arm">
        <visual>
          <origin xyz="0 0 0.5"/>
          <geometry><cylinder radius="0.05" length="1.0"/></geometry>
        </visual>
      </link>
      <joint name="shoulder" type="revolute">
        <parent link="base"/>
        <child link="upper-arm"/>
        <origin xyz="0 0 0.4" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
      </joint>
    </robot>"#;

    #[test]
    fn test_convert_simple_arm() {
        let robot = parse_urdf(SIMPLE_ARM).unwrap();
        let mut stage = Stage::new();
        convert_urdf_to_stage(&robot, &mut stage, "/arm").unwrap();

        // Articulation root and rigid-body links
        assert!(stage.has_api_schema("/arm", "PhysicsArticulationRootAPI"));
        assert!(stage.has_api_schema("/arm/base", "PhysicsRigidBodyAPI"));
        assert_eq!(stage.composed_type_name("/arm/base").unwrap(), "Xform");
        // URDF name "upper-arm" is sanitized
        assert!(stage.prim_exists("/arm/upper_arm"));

        // Child link world pose at zero configuration = joint origin
        assert_eq!(
            stage.resolve_attribute("/arm/upper_arm", "xformOp:translate"),
            Some(Value::Vec3d([0.0, 0.0, 0.4]))
        );

        // Mass: diagonal tensor passes through the eigensolver unchanged
        assert!(stage.has_api_schema("/arm/base", "PhysicsMassAPI"));
        assert_eq!(
            stage.resolve_attribute("/arm/base", "physics:mass"),
            Some(Value::Float(2.0))
        );
        assert_eq!(
            stage.resolve_attribute("/arm/base", "physics:centerOfMass"),
            Some(Value::Vec3f([0.0, 0.0, 0.1]))
        );
        assert_eq!(
            stage.resolve_attribute("/arm/base", "physics:diagonalInertia"),
            Some(Value::Vec3f([0.1, 0.2, 0.3]))
        );

        // Geometry: box as a scaled unit Cube; collision marked as guide
        assert_eq!(
            stage.composed_type_name("/arm/base/visuals/visual_0").unwrap(),
            "Cube"
        );
        assert_eq!(
            stage.resolve_attribute("/arm/base/visuals/visual_0", "xformOp:scale"),
            Some(Value::Vec3d([0.2, 0.3, 0.4]))
        );
        assert!(stage.has_api_schema("/arm/base/collisions/collision_0", "PhysicsCollisionAPI"));
        assert_eq!(
            stage.resolve_attribute("/arm/base/collisions/collision_0", "purpose"),
            Some(Value::Token("guide".to_string()))
        );
        assert_eq!(
            stage
                .composed_type_name("/arm/upper_arm/visuals/visual_0")
                .unwrap(),
            "Cylinder"
        );
        assert_eq!(
            stage.resolve_attribute("/arm/upper_arm/visuals/visual_0", "height"),
            Some(Value::Double(1.0))
        );

        // Joint: bodies, frames, axis alignment, limits in degrees
        let joint = "/arm/joints/shoulder";
        assert_eq!(stage.composed_type_name(joint).unwrap(), "PhysicsRevoluteJoint");
        assert_eq!(
            stage.relationship_targets(joint, "physics:body0"),
            Some(vec!["/arm/base".to_string()])
        );
        assert_eq!(
            stage.relationship_targets(joint, "physics:body1"),
            Some(vec!["/arm/upper_arm".to_string()])
        );
        assert_eq!(
            stage.resolve_attribute(joint, "physics:axis"),
            Some(Value::Token("X".to_string()))
        );
        let Some(Value::Float(lower)) = stage.resolve_attribute(joint, "physics:lowerLimit")
        else {
            panic!("missing lower limit")
        };
        assert!((lower - (-1.57f32).to_degrees()).abs() < 1e-3);
        // localRot1 aligns +X with the URDF Z axis
        let Some(Value::Quatf(q)) = stage.resolve_attribute(joint, "physics:localRot1") else {
            panic!("missing localRot1")
        };
        let q = [q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64];
        let r = isaacsim_core_math::transform::quaternion_to_rotation_matrix(q);
        // R * x_hat == z_hat
        assert!((r[0][0]).abs() < 1e-9 && (r[1][0]).abs() < 1e-9 && (r[2][0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_chain_world_poses_accumulate() {
        let urdf = r#"<robot name="chain">
          <link name="a"/>
          <link name="b"/>
          <link name="c"/>
          <joint name="j1" type="fixed">
            <parent link="a"/><child link="b"/>
            <origin xyz="1 0 0"/>
          </joint>
          <joint name="j2" type="fixed">
            <parent link="b"/><child link="c"/>
            <origin xyz="0 2 0" rpy="0 0 1.5707963267948966"/>
          </joint>
        </robot>"#;
        let robot = parse_urdf(urdf).unwrap();
        let mut stage = Stage::new();
        convert_urdf_to_stage(&robot, &mut stage, "/chain").unwrap();
        assert_eq!(
            stage.resolve_attribute("/chain/c", "xformOp:translate"),
            Some(Value::Vec3d([1.0, 2.0, 0.0]))
        );
        assert_eq!(
            stage.composed_type_name("/chain/joints/j1").unwrap(),
            "PhysicsFixedJoint"
        );
    }

    #[test]
    fn test_x_axis_alignment() {
        for axis in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -0.5, 0.5]] {
            let q = x_axis_alignment(axis);
            let r = isaacsim_core_math::transform::quaternion_to_rotation_matrix(q);
            let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
            for i in 0..3 {
                assert!(
                    (r[i][0] - axis[i] / norm).abs() < 1e-9,
                    "axis {axis:?}: R*x = {:?}",
                    [r[0][0], r[1][0], r[2][0]]
                );
            }
        }
    }
}
