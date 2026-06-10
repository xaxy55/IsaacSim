// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! URDF XML writer.
//!
//! Serializes a [`Robot`] model back to URDF XML, enabling the legacy
//! file-to-file pre-processing flow (`merge_fixed_joints(input, output)`).

use crate::model::*;

/// Serialize `robot` as a URDF XML document.
pub fn write_urdf(robot: &Robot) -> String {
    let mut out = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str(&format!("<robot name=\"{}\">\n", escape(&robot.name)));
    for material in &robot.materials {
        write_material(material, 1, &mut out);
    }
    for link in &robot.links {
        write_link(link, &mut out);
    }
    for joint in &robot.joints {
        write_joint(joint, &mut out);
    }
    out.push_str("</robot>\n");
    out
}

fn write_link(link: &Link, out: &mut String) {
    if link.visuals.is_empty() && link.collisions.is_empty() && link.inertial.is_none() {
        out.push_str(&format!("  <link name=\"{}\"/>\n", escape(&link.name)));
        return;
    }
    out.push_str(&format!("  <link name=\"{}\">\n", escape(&link.name)));
    if let Some(inertial) = &link.inertial {
        out.push_str("    <inertial>\n");
        write_origin(&inertial.origin, 3, out);
        out.push_str(&format!("      <mass value=\"{}\"/>\n", fmt(inertial.mass)));
        let i = &inertial.inertia;
        out.push_str(&format!(
            "      <inertia ixx=\"{}\" ixy=\"{}\" ixz=\"{}\" iyy=\"{}\" iyz=\"{}\" izz=\"{}\"/>\n",
            fmt(i.ixx),
            fmt(i.ixy),
            fmt(i.ixz),
            fmt(i.iyy),
            fmt(i.iyz),
            fmt(i.izz)
        ));
        out.push_str("    </inertial>\n");
    }
    for visual in &link.visuals {
        let name_attr = visual
            .name
            .as_ref()
            .map(|n| format!(" name=\"{}\"", escape(n)))
            .unwrap_or_default();
        out.push_str(&format!("    <visual{name_attr}>\n"));
        write_origin(&visual.origin, 3, out);
        write_geometry(&visual.geometry, out);
        if let Some(material) = &visual.material {
            write_material(material, 3, out);
        }
        out.push_str("    </visual>\n");
    }
    for collision in &link.collisions {
        let name_attr = collision
            .name
            .as_ref()
            .map(|n| format!(" name=\"{}\"", escape(n)))
            .unwrap_or_default();
        out.push_str(&format!("    <collision{name_attr}>\n"));
        write_origin(&collision.origin, 3, out);
        write_geometry(&collision.geometry, out);
        out.push_str("    </collision>\n");
    }
    out.push_str("  </link>\n");
}

fn write_joint(joint: &Joint, out: &mut String) {
    out.push_str(&format!(
        "  <joint name=\"{}\" type=\"{}\">\n",
        escape(&joint.name),
        joint.joint_type.as_str()
    ));
    out.push_str(&format!("    <parent link=\"{}\"/>\n", escape(&joint.parent)));
    out.push_str(&format!("    <child link=\"{}\"/>\n", escape(&joint.child)));
    write_origin(&joint.origin, 2, out);
    out.push_str(&format!(
        "    <axis xyz=\"{} {} {}\"/>\n",
        fmt(joint.axis[0]),
        fmt(joint.axis[1]),
        fmt(joint.axis[2])
    ));
    if let Some(limit) = &joint.limit {
        out.push_str(&format!(
            "    <limit lower=\"{}\" upper=\"{}\" effort=\"{}\" velocity=\"{}\"/>\n",
            fmt(limit.lower),
            fmt(limit.upper),
            fmt(limit.effort),
            fmt(limit.velocity)
        ));
    }
    if let Some(dynamics) = &joint.dynamics {
        out.push_str(&format!(
            "    <dynamics damping=\"{}\" friction=\"{}\"/>\n",
            fmt(dynamics.damping),
            fmt(dynamics.friction)
        ));
    }
    if let Some(mimic) = &joint.mimic {
        out.push_str(&format!(
            "    <mimic joint=\"{}\" multiplier=\"{}\" offset=\"{}\"/>\n",
            escape(&mimic.joint),
            fmt(mimic.multiplier),
            fmt(mimic.offset)
        ));
    }
    if let Some(safety) = &joint.safety_controller {
        out.push_str(&format!(
            "    <safety_controller soft_lower_limit=\"{}\" soft_upper_limit=\"{}\" \
             k_position=\"{}\" k_velocity=\"{}\"/>\n",
            fmt(safety.soft_lower_limit),
            fmt(safety.soft_upper_limit),
            fmt(safety.k_position),
            fmt(safety.k_velocity)
        ));
    }
    if let Some(calibration) = &joint.calibration {
        let rising = calibration
            .rising
            .map(|v| format!(" rising=\"{}\"", fmt(v)))
            .unwrap_or_default();
        let falling = calibration
            .falling
            .map(|v| format!(" falling=\"{}\"", fmt(v)))
            .unwrap_or_default();
        out.push_str(&format!("    <calibration{rising}{falling}/>\n"));
    }
    out.push_str("  </joint>\n");
}

fn write_geometry(geometry: &Geometry, out: &mut String) {
    out.push_str("      <geometry>\n");
    match geometry {
        Geometry::Box { size } => out.push_str(&format!(
            "        <box size=\"{} {} {}\"/>\n",
            fmt(size[0]),
            fmt(size[1]),
            fmt(size[2])
        )),
        Geometry::Cylinder { radius, length } => out.push_str(&format!(
            "        <cylinder radius=\"{}\" length=\"{}\"/>\n",
            fmt(*radius),
            fmt(*length)
        )),
        Geometry::Capsule { radius, length } => out.push_str(&format!(
            "        <capsule radius=\"{}\" length=\"{}\"/>\n",
            fmt(*radius),
            fmt(*length)
        )),
        Geometry::Sphere { radius } => {
            out.push_str(&format!("        <sphere radius=\"{}\"/>\n", fmt(*radius)))
        }
        Geometry::Mesh { filename, scale } => out.push_str(&format!(
            "        <mesh filename=\"{}\" scale=\"{} {} {}\"/>\n",
            escape(filename),
            fmt(scale[0]),
            fmt(scale[1]),
            fmt(scale[2])
        )),
    }
    out.push_str("      </geometry>\n");
}

fn write_material(material: &Material, indent: usize, out: &mut String) {
    let pad = "  ".repeat(indent);
    if material.color.is_none() && material.texture.is_none() {
        out.push_str(&format!("{pad}<material name=\"{}\"/>\n", escape(&material.name)));
        return;
    }
    out.push_str(&format!("{pad}<material name=\"{}\">\n", escape(&material.name)));
    if let Some(rgba) = material.color {
        out.push_str(&format!(
            "{pad}  <color rgba=\"{} {} {} {}\"/>\n",
            fmt(rgba[0]),
            fmt(rgba[1]),
            fmt(rgba[2]),
            fmt(rgba[3])
        ));
    }
    if let Some(texture) = &material.texture {
        out.push_str(&format!("{pad}  <texture filename=\"{}\"/>\n", escape(texture)));
    }
    out.push_str(&format!("{pad}</material>\n"));
}

fn write_origin(origin: &Origin, indent: usize, out: &mut String) {
    let pad = "  ".repeat(indent);
    out.push_str(&format!(
        "{pad}<origin xyz=\"{} {} {}\" rpy=\"{} {} {}\"/>\n",
        fmt(origin.xyz[0]),
        fmt(origin.xyz[1]),
        fmt(origin.xyz[2]),
        fmt(origin.rpy[0]),
        fmt(origin.rpy[1]),
        fmt(origin.rpy[2])
    ));
}

/// Format a float for URDF output, suppressing near-zero noise
/// (legacy `_fmt`).
fn fmt(v: f64) -> String {
    if v.abs() < 1e-12 {
        "0".to_string()
    } else {
        format!("{v}")
    }
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
