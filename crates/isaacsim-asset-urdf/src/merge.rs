// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Fixed-joint merging.
//!
//! Port of `merge_fixed_joints` from `python/impl/urdf_utils.py`, operating
//! on the parsed [`Robot`] model instead of the XML tree: for each fixed
//! joint, the child link's visuals, collisions, and inertial are merged into
//! the parent (with transform composition and the parallel axis theorem),
//! downstream joints are re-parented, and the child link is removed. Chains
//! of fixed joints collapse by iterating until none remain.
//!
//! Unlike the legacy XML-level pass, non-standard extension elements
//! (`<transmission>`, `<gazebo>`, …) are not carried through — the parser
//! already drops them at the port boundary.

use isaacsim_core_math::linalg::mat4_mul;

use crate::model::{Inertia, Inertial, Origin, Robot};
use crate::transform::{compose_origins, mat3_mul, mat3_transpose, origin_to_matrix};

/// Merge links connected by fixed joints into their parents, in place.
///
/// Malformed fixed joints (self-loops, references to missing links) are
/// dropped without merging, matching the legacy behavior of logging and
/// removing the joint.
pub fn merge_fixed_joints(robot: &mut Robot) {
    loop {
        let Some(joint_idx) = robot
            .joints
            .iter()
            .position(|j| j.joint_type == crate::model::JointType::Fixed)
        else {
            break;
        };
        let joint = robot.joints.remove(joint_idx);
        let parent_name = joint.parent.clone();
        let child_name = joint.child.clone();

        // Self-loops and missing links: drop the joint without merging
        if parent_name == child_name
            || robot.link(&parent_name).is_none()
            || robot.link(&child_name).is_none()
        {
            continue;
        }

        let t_joint = origin_to_matrix(&joint.origin);
        let child_pos = robot
            .links
            .iter()
            .position(|l| l.name == child_name)
            .expect("checked above");
        let child = robot.links.remove(child_pos);

        // Move visuals and collisions to the parent with composed transforms
        let parent = robot.link_mut(&parent_name).expect("checked above");
        for mut visual in child.visuals {
            visual.origin = compose_origins(&t_joint, &visual.origin);
            parent.visuals.push(visual);
        }
        for mut collision in child.collisions {
            collision.origin = compose_origins(&t_joint, &collision.origin);
            parent.collisions.push(collision);
        }

        // Merge inertial properties (mass, CoM, inertia tensor)
        if let Some(child_inertial) = child.inertial {
            merge_inertial(parent, &child_inertial, &t_joint);
        }

        // Re-parent downstream joints with composed origins
        for other in &mut robot.joints {
            if other.parent == child_name {
                other.parent = parent_name.clone();
                let t_other = origin_to_matrix(&other.origin);
                other.origin = crate::transform::matrix_to_origin(&mat4_mul(&t_joint, &t_other));
            }
        }
    }
}

fn inertia_matrix(inertia: &Inertia) -> [[f64; 3]; 3] {
    [
        [inertia.ixx, inertia.ixy, inertia.ixz],
        [inertia.ixy, inertia.iyy, inertia.iyz],
        [inertia.ixz, inertia.iyz, inertia.izz],
    ]
}

fn rotation_part(t: &[[f64; 4]; 4]) -> [[f64; 3]; 3] {
    [
        [t[0][0], t[0][1], t[0][2]],
        [t[1][0], t[1][1], t[1][2]],
        [t[2][0], t[2][1], t[2][2]],
    ]
}

/// Port of legacy `_merge_inertial`: parallel axis theorem combination of
/// two rigidly attached bodies, written back as the parent's inertial with
/// the combined CoM and an identity-rotation origin.
fn merge_inertial(parent: &mut crate::model::Link, child_inertial: &Inertial, t_joint: &[[f64; 4]; 4]) {
    if child_inertial.mass == 0.0 {
        return; // zero mass — nothing to merge
    }

    // Child inertial in the parent link frame
    let t_child_in_parent = mat4_mul(t_joint, &origin_to_matrix(&child_inertial.origin));
    let r_child = rotation_part(&t_child_in_parent);
    let child_com = [
        t_child_in_parent[0][3],
        t_child_in_parent[1][3],
        t_child_in_parent[2][3],
    ];
    let child_i_in_parent = mat3_mul(
        &mat3_mul(&r_child, &inertia_matrix(&child_inertial.inertia)),
        &mat3_transpose(&r_child),
    );

    // Parent inertial (defaults to an empty body)
    let parent_inertial = parent.inertial.unwrap_or_default();
    let parent_mass = parent_inertial.mass;
    let t_parent = origin_to_matrix(&parent_inertial.origin);
    let r_parent = rotation_part(&t_parent);
    let parent_com = parent_inertial.origin.xyz;
    let parent_i_in_link = mat3_mul(
        &mat3_mul(&r_parent, &inertia_matrix(&parent_inertial.inertia)),
        &mat3_transpose(&r_parent),
    );

    let total_mass = parent_mass + child_inertial.mass;
    if total_mass == 0.0 {
        return;
    }
    let combined_com = [
        (parent_mass * parent_com[0] + child_inertial.mass * child_com[0]) / total_mass,
        (parent_mass * parent_com[1] + child_inertial.mass * child_com[1]) / total_mass,
        (parent_mass * parent_com[2] + child_inertial.mass * child_com[2]) / total_mass,
    ];

    // Parallel axis theorem: shift each tensor to the combined CoM
    let shift = |i_at_com: [[f64; 3]; 3], mass: f64, com: [f64; 3]| {
        let d = [
            combined_com[0] - com[0],
            combined_com[1] - com[1],
            combined_com[2] - com[2],
        ];
        let d_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        let mut out = i_at_com;
        for (r, row) in out.iter_mut().enumerate() {
            for (c, value) in row.iter_mut().enumerate() {
                let identity = if r == c { 1.0 } else { 0.0 };
                *value += mass * (d_sq * identity - d[r] * d[c]);
            }
        }
        out
    };

    let parent_shifted = if parent_mass > 0.0 {
        shift(parent_i_in_link, parent_mass, parent_com)
    } else {
        parent_i_in_link
    };
    let child_shifted = shift(child_i_in_parent, child_inertial.mass, child_com);

    let mut combined = [[0.0; 3]; 3];
    for (r, row) in combined.iter_mut().enumerate() {
        for (c, value) in row.iter_mut().enumerate() {
            *value = parent_shifted[r][c] + child_shifted[r][c];
        }
    }

    parent.inertial = Some(Inertial {
        origin: Origin {
            xyz: combined_com,
            rpy: [0.0, 0.0, 0.0],
        },
        mass: total_mass,
        inertia: Inertia {
            ixx: combined[0][0],
            ixy: combined[0][1],
            ixz: combined[0][2],
            iyy: combined[1][1],
            iyz: combined[1][2],
            izz: combined[2][2],
        },
    });
}
