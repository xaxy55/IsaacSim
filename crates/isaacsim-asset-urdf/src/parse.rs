// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! URDF XML parser.
//!
//! Parses a URDF document into the [`Robot`] model. Standard elements are
//! fully parsed; non-standard extensions (`<transmission>`, `<gazebo>`,
//! `<sensor>`, …) are skipped, matching the legacy parsing layer's port
//! boundary (USD conversion and physics setup are downstream concerns).

use roxmltree::{Document, Node};

use crate::model::*;

/// Parse a URDF document from XML text.
pub fn parse_urdf(text: &str) -> Result<Robot, String> {
    let doc = Document::parse(text).map_err(|e| format!("invalid XML: {e}"))?;
    let root = doc.root_element();
    if root.tag_name().name() != "robot" {
        return Err(format!(
            "expected <robot> root element, got <{}>",
            root.tag_name().name()
        ));
    }

    let mut robot = Robot {
        name: root.attribute("name").unwrap_or_default().to_string(),
        ..Default::default()
    };

    for node in root.children().filter(Node::is_element) {
        match node.tag_name().name() {
            "link" => robot.links.push(parse_link(node)?),
            "joint" => robot.joints.push(parse_joint(node)?),
            "material" => robot.materials.push(parse_material(node)),
            // transmission/gazebo/sensor and other extensions are skipped
            _ => {}
        }
    }
    Ok(robot)
}

/// Parse a URDF file from disk.
pub fn parse_urdf_file(path: impl AsRef<std::path::Path>) -> Result<Robot, String> {
    let text = std::fs::read_to_string(path.as_ref())
        .map_err(|e| format!("failed to read {}: {e}", path.as_ref().display()))?;
    parse_urdf(&text)
}

fn parse_link(node: Node) -> Result<Link, String> {
    let name = require_attr(node, "name")?;
    let mut link = Link {
        name,
        ..Default::default()
    };
    for child in node.children().filter(Node::is_element) {
        match child.tag_name().name() {
            "visual" => link.visuals.push(Visual {
                name: child.attribute("name").map(str::to_string),
                origin: parse_origin(find(child, "origin"))?,
                geometry: parse_geometry(child)?,
                material: find(child, "material").map(parse_material),
            }),
            "collision" => link.collisions.push(Collision {
                name: child.attribute("name").map(str::to_string),
                origin: parse_origin(find(child, "origin"))?,
                geometry: parse_geometry(child)?,
            }),
            "inertial" => link.inertial = Some(parse_inertial(child)?),
            _ => {}
        }
    }
    Ok(link)
}

fn parse_joint(node: Node) -> Result<Joint, String> {
    let name = require_attr(node, "name")?;
    let type_str = node
        .attribute("type")
        .ok_or_else(|| format!("joint {name:?} has no type attribute"))?;
    let joint_type = match type_str {
        "fixed" => JointType::Fixed,
        "revolute" => JointType::Revolute,
        "continuous" => JointType::Continuous,
        "prismatic" => JointType::Prismatic,
        "floating" => JointType::Floating,
        "planar" => JointType::Planar,
        other => return Err(format!("joint {name:?} has unknown type {other:?}")),
    };
    let parent = find(node, "parent")
        .and_then(|n| n.attribute("link"))
        .ok_or_else(|| format!("joint {name:?} has no <parent link=...>"))?
        .to_string();
    let child = find(node, "child")
        .and_then(|n| n.attribute("link"))
        .ok_or_else(|| format!("joint {name:?} has no <child link=...>"))?
        .to_string();

    Ok(Joint {
        origin: parse_origin(find(node, "origin"))?,
        axis: match find(node, "axis") {
            Some(axis) => parse_vec3(axis.attribute("xyz").unwrap_or("1 0 0"))?,
            None => [1.0, 0.0, 0.0],
        },
        limit: find(node, "limit")
            .map(|n| {
                Ok::<_, String>(JointLimit {
                    lower: attr_f64(n, "lower", 0.0)?,
                    upper: attr_f64(n, "upper", 0.0)?,
                    effort: attr_f64(n, "effort", 0.0)?,
                    velocity: attr_f64(n, "velocity", 0.0)?,
                })
            })
            .transpose()?,
        dynamics: find(node, "dynamics")
            .map(|n| {
                Ok::<_, String>(JointDynamics {
                    damping: attr_f64(n, "damping", 0.0)?,
                    friction: attr_f64(n, "friction", 0.0)?,
                })
            })
            .transpose()?,
        mimic: find(node, "mimic")
            .map(|n| {
                Ok::<_, String>(JointMimic {
                    joint: require_attr(n, "joint")?,
                    multiplier: attr_f64(n, "multiplier", 1.0)?,
                    offset: attr_f64(n, "offset", 0.0)?,
                })
            })
            .transpose()?,
        safety_controller: find(node, "safety_controller")
            .map(|n| {
                Ok::<_, String>(SafetyController {
                    soft_lower_limit: attr_f64(n, "soft_lower_limit", 0.0)?,
                    soft_upper_limit: attr_f64(n, "soft_upper_limit", 0.0)?,
                    k_position: attr_f64(n, "k_position", 0.0)?,
                    k_velocity: attr_f64(n, "k_velocity", 0.0)?,
                })
            })
            .transpose()?,
        calibration: find(node, "calibration")
            .map(|n| {
                Ok::<_, String>(JointCalibration {
                    rising: opt_attr_f64(n, "rising")?,
                    falling: opt_attr_f64(n, "falling")?,
                })
            })
            .transpose()?,
        name,
        joint_type,
        parent,
        child,
    })
}

fn parse_inertial(node: Node) -> Result<Inertial, String> {
    let inertia = match find(node, "inertia") {
        Some(n) => Inertia {
            ixx: attr_f64(n, "ixx", 0.0)?,
            ixy: attr_f64(n, "ixy", 0.0)?,
            ixz: attr_f64(n, "ixz", 0.0)?,
            iyy: attr_f64(n, "iyy", 0.0)?,
            iyz: attr_f64(n, "iyz", 0.0)?,
            izz: attr_f64(n, "izz", 0.0)?,
        },
        None => Inertia::default(),
    };
    Ok(Inertial {
        origin: parse_origin(find(node, "origin"))?,
        mass: match find(node, "mass") {
            Some(n) => attr_f64(n, "value", 0.0)?,
            None => 0.0,
        },
        inertia,
    })
}

fn parse_geometry(parent: Node) -> Result<Geometry, String> {
    let geometry = find(parent, "geometry")
        .ok_or_else(|| format!("<{}> has no <geometry>", parent.tag_name().name()))?;
    let shape = geometry
        .children()
        .find(Node::is_element)
        .ok_or("empty <geometry>")?;
    match shape.tag_name().name() {
        "box" => Ok(Geometry::Box {
            size: parse_vec3(shape.attribute("size").unwrap_or("0 0 0"))?,
        }),
        "cylinder" => Ok(Geometry::Cylinder {
            radius: attr_f64(shape, "radius", 0.0)?,
            length: attr_f64(shape, "length", 0.0)?,
        }),
        "capsule" => Ok(Geometry::Capsule {
            radius: attr_f64(shape, "radius", 0.0)?,
            length: attr_f64(shape, "length", 0.0)?,
        }),
        "sphere" => Ok(Geometry::Sphere {
            radius: attr_f64(shape, "radius", 0.0)?,
        }),
        "mesh" => Ok(Geometry::Mesh {
            filename: shape.attribute("filename").unwrap_or_default().to_string(),
            scale: match shape.attribute("scale") {
                Some(scale) => parse_vec3(scale)?,
                None => [1.0, 1.0, 1.0],
            },
        }),
        other => Err(format!("unknown geometry shape <{other}>")),
    }
}

fn parse_material(node: Node) -> Material {
    Material {
        name: node.attribute("name").unwrap_or_default().to_string(),
        color: find(node, "color")
            .and_then(|n| n.attribute("rgba"))
            .and_then(|rgba| {
                let v: Result<Vec<f64>, _> =
                    rgba.split_whitespace().map(str::parse).collect();
                match v {
                    Ok(v) if v.len() == 4 => Some([v[0], v[1], v[2], v[3]]),
                    _ => None,
                }
            }),
        texture: find(node, "texture")
            .and_then(|n| n.attribute("filename"))
            .map(str::to_string),
    }
}

/// Parse an `<origin>` element (legacy `_parse_origin`; `None` is identity).
fn parse_origin(node: Option<Node>) -> Result<Origin, String> {
    let Some(node) = node else {
        return Ok(Origin::default());
    };
    Ok(Origin {
        xyz: parse_vec3(node.attribute("xyz").unwrap_or("0 0 0"))?,
        rpy: parse_vec3(node.attribute("rpy").unwrap_or("0 0 0"))?,
    })
}

fn parse_vec3(text: &str) -> Result<[f64; 3], String> {
    let values: Result<Vec<f64>, _> = text.split_whitespace().map(str::parse).collect();
    let values = values.map_err(|e| format!("invalid vector {text:?}: {e}"))?;
    if values.len() != 3 {
        return Err(format!("expected 3 values, got {} in {text:?}", values.len()));
    }
    Ok([values[0], values[1], values[2]])
}

fn find<'a>(node: Node<'a, 'a>, name: &str) -> Option<Node<'a, 'a>> {
    node.children()
        .find(|c| c.is_element() && c.tag_name().name() == name)
}

fn require_attr(node: Node, name: &str) -> Result<String, String> {
    node.attribute(name)
        .map(str::to_string)
        .ok_or_else(|| format!("<{}> has no {name:?} attribute", node.tag_name().name()))
}

fn attr_f64(node: Node, name: &str, default: f64) -> Result<f64, String> {
    match node.attribute(name) {
        None => Ok(default),
        Some(text) => text
            .trim()
            .parse()
            .map_err(|e| format!("invalid {name}={text:?}: {e}")),
    }
}

fn opt_attr_f64(node: Node, name: &str) -> Result<Option<f64>, String> {
    match node.attribute(name) {
        None => Ok(None),
        Some(text) => text
            .trim()
            .parse()
            .map(Some)
            .map_err(|e| format!("invalid {name}={text:?}: {e}")),
    }
}
