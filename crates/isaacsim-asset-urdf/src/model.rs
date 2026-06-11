// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! URDF robot data model.
//!
//! In-memory representation of a parsed URDF document: robot, links, joints,
//! visuals/collisions, inertials, and materials. Field defaults follow the
//! URDF specification (e.g. joint axis `(1, 0, 0)`, identity origins).

/// Pose of an element: translation `xyz` plus fixed-axis `rpy` Euler angles
/// in radians (URDF convention, rotation applied as `Rz * Ry * Rx`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Origin {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3],
}

/// Symmetric 3x3 rotational inertia tensor components.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

/// `<inertial>`: mass properties of a link.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Inertial {
    /// Pose of the center of mass relative to the link frame.
    pub origin: Origin,
    /// Mass in kg.
    pub mass: f64,
    /// Rotational inertia about the center of mass, in the origin frame.
    pub inertia: Inertia,
}

/// `<geometry>` shapes.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    Box { size: [f64; 3] },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { filename: String, scale: [f64; 3] },
}

/// `<material>`: name plus optional color and texture.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Material {
    pub name: String,
    /// RGBA color in [0, 1].
    pub color: Option<[f64; 4]>,
    /// Texture image filename.
    pub texture: Option<String>,
}

/// `<visual>`: a visual geometry of a link.
#[derive(Debug, Clone, PartialEq)]
pub struct Visual {
    pub name: Option<String>,
    pub origin: Origin,
    pub geometry: Geometry,
    pub material: Option<Material>,
}

/// `<collision>`: a collision geometry of a link.
#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    pub name: Option<String>,
    pub origin: Origin,
    pub geometry: Geometry,
}

/// `<link>`: a rigid body.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Link {
    pub name: String,
    pub visuals: Vec<Visual>,
    pub collisions: Vec<Collision>,
    pub inertial: Option<Inertial>,
}

/// `<joint type="...">` values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar,
}

impl JointType {
    /// The URDF type attribute string.
    pub fn as_str(&self) -> &'static str {
        match self {
            JointType::Fixed => "fixed",
            JointType::Revolute => "revolute",
            JointType::Continuous => "continuous",
            JointType::Prismatic => "prismatic",
            JointType::Floating => "floating",
            JointType::Planar => "planar",
        }
    }
}

/// `<limit>`: position bounds in rad (or m), max effort in N·m (or N), and
/// max velocity in rad/s (or m/s).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointLimit {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

/// `<dynamics>`: physical damping and friction of the joint.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointDynamics {
    pub damping: f64,
    pub friction: f64,
}

/// `<mimic>`: this joint's position is
/// `multiplier * mimicked_joint_position + offset`.
#[derive(Debug, Clone, PartialEq)]
pub struct JointMimic {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// `<safety_controller>` parameters.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SafetyController {
    pub soft_lower_limit: f64,
    pub soft_upper_limit: f64,
    pub k_position: f64,
    pub k_velocity: f64,
}

/// `<calibration>` reference positions.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointCalibration {
    pub rising: Option<f64>,
    pub falling: Option<f64>,
}

/// `<joint>`: a connection between two links.
#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    /// Parent link name.
    pub parent: String,
    /// Child link name.
    pub child: String,
    /// Transform from the parent link frame to the child link frame.
    pub origin: Origin,
    /// Joint axis in the joint frame (URDF default `(1, 0, 0)`).
    pub axis: [f64; 3],
    pub limit: Option<JointLimit>,
    pub dynamics: Option<JointDynamics>,
    pub mimic: Option<JointMimic>,
    pub safety_controller: Option<SafetyController>,
    pub calibration: Option<JointCalibration>,
}

/// `<robot>`: the root of a URDF document.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Robot {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    /// Top-level material definitions (referenced by name from visuals).
    pub materials: Vec<Material>,
}

impl Robot {
    /// Find a link by name.
    pub fn link(&self, name: &str) -> Option<&Link> {
        self.links.iter().find(|l| l.name == name)
    }

    /// Find a link by name (mutable).
    pub fn link_mut(&mut self, name: &str) -> Option<&mut Link> {
        self.links.iter_mut().find(|l| l.name == name)
    }

    /// Find a joint by name.
    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|j| j.name == name)
    }
}
