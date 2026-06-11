// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Stage → URDF export (rigid-body subset).
//!
//! Port of the core of the legacy `isaacsim.asset.exporter.urdf` converter:
//! reads a UsdPhysics articulation from any [`SceneStage`] back into the
//! [`Robot`] model — the inverse of
//! [`convert_urdf_to_stage`](crate::convert::convert_urdf_to_stage).
//!
//! Joint origins and axes are reconstructed from the UsdPhysics joint
//! frames: `axis = R(localRot1) * x̂` and
//! `q_origin = localRot0 * localRot1⁻¹`; revolute limits convert from
//! degrees back to radians. Inertia tensors are rebuilt from
//! `diagonalInertia` + `principalAxes` with an identity inertial-origin
//! rotation (semantically equal mass properties).
//!
//! Out of scope, as in the converter: mesh tessellation (mesh prims carry
//! the source filename), drives (URDF `effort`/`velocity` export as 0),
//! sensors, and cameras.

use isaacsim_core_math::transform::{
    quaternion_conjugate, quaternion_multiplication, quaternion_to_euler_angles,
    quaternion_to_rotation_matrix,
};
use isaacsim_scene::{descendants, SceneStage, Value};

use crate::model::*;

fn leaf(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

fn attr_f64<S: SceneStage + ?Sized>(stage: &S, path: &str, name: &str) -> Option<f64> {
    match stage.attribute(path, name)? {
        Value::Float(v) => Some(v as f64),
        Value::Double(v) => Some(v),
        Value::Int(v) => Some(v as f64),
        _ => None,
    }
}

fn quaternion_to_rpy(q: [f64; 4]) -> [f64; 3] {
    // URDF rpy is fixed-axis (extrinsic XYZ): Rz * Ry * Rx
    quaternion_to_euler_angles(q, false, true)
}

fn read_origin<S: SceneStage + ?Sized>(stage: &S, path: &str) -> Origin {
    let xyz = stage
        .attribute(path, "xformOp:translate")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([0.0, 0.0, 0.0]);
    let orient = stage
        .attribute(path, "xformOp:orient")
        .and_then(|v| v.as_quatd())
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    Origin {
        xyz,
        rpy: quaternion_to_rpy(orient),
    }
}

/// Read a UsdPhysics articulation rooted at `root_path` back into a
/// [`Robot`] named `robot_name`.
///
/// Links are the direct children of `root_path` carrying
/// `PhysicsRigidBodyAPI`; joints are `Physics*Joint`-typed prims anywhere in
/// the subtree. Errors if no links are found.
pub fn export_stage_to_urdf<S: SceneStage + ?Sized>(
    stage: &S,
    root_path: &str,
    robot_name: &str,
) -> Result<Robot, String> {
    let mut robot = Robot {
        name: robot_name.to_string(),
        ..Default::default()
    };

    for link_path in stage.children(root_path) {
        if !stage.has_api_schema(&link_path, "PhysicsRigidBodyAPI") {
            continue;
        }
        robot.links.push(read_link(stage, &link_path)?);
    }
    if robot.links.is_empty() {
        return Err(format!("no PhysicsRigidBodyAPI links found under {root_path}"));
    }

    for path in descendants(stage, root_path) {
        let Some(type_name) = stage.type_name(&path) else {
            continue;
        };
        if type_name.starts_with("Physics") && type_name.ends_with("Joint") {
            robot.joints.push(read_joint(stage, &path, &type_name)?);
        }
    }
    Ok(robot)
}

fn read_link<S: SceneStage + ?Sized>(stage: &S, link_path: &str) -> Result<Link, String> {
    let mut link = Link {
        name: leaf(link_path).to_string(),
        ..Default::default()
    };

    if stage.has_api_schema(link_path, "PhysicsMassAPI") {
        link.inertial = Some(read_inertial(stage, link_path));
    }

    for child in stage.children(&format!("{link_path}/visuals")) {
        link.visuals.push(Visual {
            name: Some(leaf(&child).to_string()),
            origin: read_origin(stage, &child),
            geometry: read_geometry(stage, &child)?,
            material: None,
        });
    }
    for child in stage.children(&format!("{link_path}/collisions")) {
        link.collisions.push(Collision {
            name: Some(leaf(&child).to_string()),
            origin: read_origin(stage, &child),
            geometry: read_geometry(stage, &child)?,
        });
    }
    Ok(link)
}

fn read_inertial<S: SceneStage + ?Sized>(stage: &S, link_path: &str) -> Inertial {
    let mass = attr_f64(stage, link_path, "physics:mass").unwrap_or(0.0);
    let com = stage
        .attribute(link_path, "physics:centerOfMass")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([0.0, 0.0, 0.0]);
    let diagonal = stage
        .attribute(link_path, "physics:diagonalInertia")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([0.0, 0.0, 0.0]);
    let principal = stage
        .attribute(link_path, "physics:principalAxes")
        .and_then(|v| v.as_quatd())
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);

    // I = R diag R^T with the principal-axes rotation
    let r = quaternion_to_rotation_matrix(principal);
    let mut tensor = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            tensor[i][j] = (0..3).map(|k| r[i][k] * diagonal[k] * r[j][k]).sum();
        }
    }
    Inertial {
        origin: Origin {
            xyz: com,
            rpy: [0.0, 0.0, 0.0],
        },
        mass,
        inertia: Inertia {
            ixx: tensor[0][0],
            ixy: tensor[0][1],
            ixz: tensor[0][2],
            iyy: tensor[1][1],
            iyz: tensor[1][2],
            izz: tensor[2][2],
        },
    }
}

fn read_geometry<S: SceneStage + ?Sized>(stage: &S, path: &str) -> Result<Geometry, String> {
    let type_name = stage
        .type_name(path)
        .ok_or_else(|| format!("geometry prim {path} does not exist"))?;
    let scale = stage
        .attribute(path, "xformOp:scale")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([1.0, 1.0, 1.0]);
    match type_name.as_str() {
        // The converter authors boxes as unit Cubes scaled to the box size
        "Cube" => Ok(Geometry::Box { size: scale }),
        "Sphere" => Ok(Geometry::Sphere {
            radius: attr_f64(stage, path, "radius").unwrap_or(0.0),
        }),
        "Cylinder" => Ok(Geometry::Cylinder {
            radius: attr_f64(stage, path, "radius").unwrap_or(0.0),
            length: attr_f64(stage, path, "height").unwrap_or(0.0),
        }),
        "Capsule" => Ok(Geometry::Capsule {
            radius: attr_f64(stage, path, "radius").unwrap_or(0.0),
            length: attr_f64(stage, path, "height").unwrap_or(0.0),
        }),
        "Mesh" => Ok(Geometry::Mesh {
            filename: match stage.attribute(path, "isaac:urdf:meshFile") {
                Some(Value::String(filename)) => filename,
                _ => String::new(),
            },
            scale,
        }),
        other => Err(format!("unsupported geometry prim type {other:?} at {path}")),
    }
}

fn read_joint<S: SceneStage + ?Sized>(
    stage: &S,
    path: &str,
    type_name: &str,
) -> Result<Joint, String> {
    let body = |rel: &str| -> Result<String, String> {
        stage
            .relationship_targets(path, rel)
            .and_then(|targets| targets.into_iter().next())
            .map(|target| leaf(&target).to_string())
            .ok_or_else(|| format!("joint {path} has no {rel} target"))
    };
    let parent = body("physics:body0")?;
    let child = body("physics:body1")?;

    let local_pos0 = stage
        .attribute(path, "physics:localPos0")
        .and_then(|v| v.as_vec3d())
        .unwrap_or([0.0, 0.0, 0.0]);
    let local_rot0 = stage
        .attribute(path, "physics:localRot0")
        .and_then(|v| v.as_quatd())
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let local_rot1 = stage
        .attribute(path, "physics:localRot1")
        .and_then(|v| v.as_quatd())
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);

    // Invert the converter's axis fold: the joint frame aligns +X with the
    // URDF axis, so axis = R(localRot1) * x_hat and
    // q_origin = localRot0 * localRot1^-1
    let r1 = quaternion_to_rotation_matrix(local_rot1);
    let axis = [r1[0][0], r1[1][0], r1[2][0]];
    let q_origin = quaternion_multiplication(local_rot0, quaternion_conjugate(local_rot1));
    let origin = Origin {
        xyz: local_pos0,
        rpy: quaternion_to_rpy(q_origin),
    };

    let limit = |degrees: bool| -> Option<JointLimit> {
        let lower = attr_f64(stage, path, "physics:lowerLimit")?;
        let upper = attr_f64(stage, path, "physics:upperLimit")?;
        let (lower, upper) = if degrees {
            (lower.to_radians(), upper.to_radians())
        } else {
            (lower, upper)
        };
        // Drives are out of scope: effort/velocity are not represented in
        // the UsdPhysics joint and export as 0
        Some(JointLimit {
            lower,
            upper,
            effort: 0.0,
            velocity: 0.0,
        })
    };

    let (joint_type, limit, axis) = match type_name {
        "PhysicsFixedJoint" => (JointType::Fixed, None, [1.0, 0.0, 0.0]),
        // Limitless revolute joints are continuous
        "PhysicsRevoluteJoint" => match limit(true) {
            Some(l) => (JointType::Revolute, Some(l), axis),
            None => (JointType::Continuous, None, axis),
        },
        "PhysicsPrismaticJoint" => (JointType::Prismatic, limit(false), axis),
        other => return Err(format!("unsupported joint prim type {other:?} at {path}")),
    };

    Ok(Joint {
        name: leaf(path).to_string(),
        joint_type,
        parent,
        child,
        origin,
        axis,
        limit,
        dynamics: None,
        mimic: None,
        safety_controller: None,
        calibration: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert::convert_urdf_to_stage;
    use crate::parse::parse_urdf;
    use isaacsim_scene::Stage;

    fn assert_origin_close(a: &Origin, b: &Origin, context: &str) {
        // 1e-6 tolerance: physics attributes are authored in float32
        for (x, y) in a.xyz.iter().zip(&b.xyz) {
            assert!((x - y).abs() < 1e-6, "{context}: xyz {:?} vs {:?}", a.xyz, b.xyz);
        }
        for (x, y) in a.rpy.iter().zip(&b.rpy) {
            assert!((x - y).abs() < 1e-6, "{context}: rpy {:?} vs {:?}", a.rpy, b.rpy);
        }
    }

    /// URDF -> stage -> URDF round trip preserves the rigid-body model.
    #[test]
    fn test_round_trip() {
        let urdf = r#"<robot name="rt">
          <link name="base">
            <inertial>
              <mass value="2.5"/>
              <origin xyz="0 0.1 0.2"/>
              <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0.01" ixz="0" iyz="0.02"/>
            </inertial>
            <visual>
              <origin xyz="0.1 0 0" rpy="0.2 0.3 0.4"/>
              <geometry><box size="0.2 0.3 0.4"/></geometry>
            </visual>
            <collision>
              <geometry><capsule radius="0.1" length="0.5"/></geometry>
            </collision>
          </link>
          <link name="arm">
            <visual><geometry><cylinder radius="0.05" length="1.0"/></geometry></visual>
          </link>
          <link name="slider">
            <visual><geometry><sphere radius="0.03"/></geometry></visual>
          </link>
          <joint name="shoulder" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <origin xyz="0 0 0.4" rpy="0 0.1 0.5"/>
            <axis xyz="0 0.6 0.8"/>
            <limit lower="-1.2" upper="0.9" effort="10" velocity="2"/>
          </joint>
          <joint name="wrist" type="continuous">
            <parent link="arm"/>
            <child link="slider"/>
            <origin xyz="0 0 1.0"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>"#;
        let original = parse_urdf(urdf).unwrap();
        let mut stage = Stage::new();
        convert_urdf_to_stage(&original, &mut stage, "/rt").unwrap();
        let exported = export_stage_to_urdf(&stage, "/rt", "rt").unwrap();

        assert_eq!(exported.name, "rt");
        assert_eq!(exported.links.len(), 3);
        assert_eq!(exported.joints.len(), 2);

        // Links: geometry shapes and origins survive
        let base = exported.link("base").unwrap();
        let original_base = original.link("base").unwrap();
        assert_eq!(base.visuals.len(), 1);
        assert_eq!(base.visuals[0].geometry, original_base.visuals[0].geometry);
        assert_origin_close(
            &base.visuals[0].origin,
            &original_base.visuals[0].origin,
            "base visual",
        );
        assert_eq!(base.collisions[0].geometry, original_base.collisions[0].geometry);

        // Mass properties: mass, CoM, and the full tensor (reconstructed
        // from diagonal + principal axes) match within float32 authoring
        let inertial = base.inertial.unwrap();
        let original_inertial = original_base.inertial.unwrap();
        assert!((inertial.mass - original_inertial.mass).abs() < 1e-6);
        assert_origin_close(&inertial.origin, &original_inertial.origin, "inertial");
        for (a, b) in [
            (inertial.inertia.ixx, original_inertial.inertia.ixx),
            (inertial.inertia.ixy, original_inertial.inertia.ixy),
            (inertial.inertia.ixz, original_inertial.inertia.ixz),
            (inertial.inertia.iyy, original_inertial.inertia.iyy),
            (inertial.inertia.iyz, original_inertial.inertia.iyz),
            (inertial.inertia.izz, original_inertial.inertia.izz),
        ] {
            assert!((a - b).abs() < 1e-6, "inertia {a} vs {b}");
        }

        // Joints: type, topology, origin, axis, and limits in radians
        let shoulder = exported.joint("shoulder").unwrap();
        let original_shoulder = original.joint("shoulder").unwrap();
        assert_eq!(shoulder.joint_type, JointType::Revolute);
        assert_eq!(shoulder.parent, "base");
        assert_eq!(shoulder.child, "arm");
        assert_origin_close(&shoulder.origin, &original_shoulder.origin, "shoulder origin");
        // The recovered axis is the normalized original
        for (a, b) in shoulder.axis.iter().zip(&[0.0, 0.6, 0.8]) {
            assert!((a - b).abs() < 1e-6, "axis {:?}", shoulder.axis);
        }
        let limit = shoulder.limit.unwrap();
        let original_limit = original_shoulder.limit.unwrap();
        assert!((limit.lower - original_limit.lower).abs() < 1e-6);
        assert!((limit.upper - original_limit.upper).abs() < 1e-6);

        // Continuous joints come back limitless
        let wrist = exported.joint("wrist").unwrap();
        assert_eq!(wrist.joint_type, JointType::Continuous);
        assert!(wrist.limit.is_none());
        for (a, b) in wrist.axis.iter().zip(&[0.0, 1.0, 0.0]) {
            assert!((a - b).abs() < 1e-6, "axis {:?}", wrist.axis);
        }

        // The exported model serializes to valid URDF and re-parses
        let reparsed = parse_urdf(&crate::write::write_urdf(&exported)).unwrap();
        assert_eq!(reparsed.links.len(), 3);
        assert_eq!(reparsed.joints.len(), 2);
    }

    #[test]
    fn test_export_missing_links_fails() {
        let stage = Stage::new();
        assert!(export_stage_to_urdf(&stage, "/nothing", "r").is_err());
    }
}
