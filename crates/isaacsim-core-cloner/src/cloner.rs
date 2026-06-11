// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Cloner module.
//!
//! Port of `python/impl/cloner.py` from the legacy `isaacsim.core.cloner`
//! extension. The PhysX-specific paths (`replicate_physics`,
//! `filter_collisions`, Fabric cloning, notice-handler toggling) depend on
//! closed-source components and are out of scope; see ROADMAP.md.

use isaacsim_scene::{is_valid_path_string, SceneStage, Value};

/// Options for [`Cloner::clone`]. Quaternions are `[w, x, y, z]`.
#[derive(Debug, Clone, Default)]
pub struct CloneOptions {
    /// Target positions of clones, one per prim path. `None` keeps the
    /// source translation for every clone.
    pub positions: Option<Vec<[f64; 3]>>,
    /// Target orientations of clones, one per prim path. `None` keeps the
    /// source orientation for every clone.
    pub orientations: Option<Vec<[f64; 4]>>,
    /// `false` (default) makes clones inherit from the source prim, so later
    /// changes to the source are reflected in the clones. `true` copies the
    /// source spec instead, making clones independent.
    pub copy_from_source: bool,
}

/// Simple APIs to duplicate objects at user-specified locations in the scene.
///
/// Cloning is performed in a for-loop, so performance follows linear scaling
/// with the number of clones.
///
/// Note: deliberately not `Clone` — the legacy-named [`Self::clone`] prim
/// duplication method would otherwise be shadowed by `Clone::clone`.
#[derive(Debug, Default)]
pub struct Cloner {
    base_env_path: Option<String>,
    root_path: Option<String>,
}

impl Cloner {
    /// Create a cloner.
    pub fn new() -> Self {
        Self::default()
    }

    /// The base environment path set by [`Self::define_base_env`].
    pub fn base_env_path(&self) -> Option<&str> {
        self.base_env_path.as_deref()
    }

    /// The root path prefix recorded by [`Self::generate_paths`].
    pub fn root_path(&self) -> Option<&str> {
        self.root_path.as_deref()
    }

    /// Create a `Scope` prim at `base_env_path`, designed to be the parent
    /// that holds all clones.
    pub fn define_base_env<S: SceneStage + ?Sized>(
        &mut self,
        stage: &mut S,
        base_env_path: &str,
    ) -> Result<(), String> {
        stage.define_prim(base_env_path, "Scope")?;
        self.base_env_path = Some(base_env_path.to_string());
        Ok(())
    }

    /// Generate `num_paths` paths in the format `{root_path}_{i}`.
    pub fn generate_paths(&mut self, root_path: &str, num_paths: usize) -> Vec<String> {
        self.root_path = Some(format!("{root_path}_"));
        (0..num_paths).map(|i| format!("{root_path}_{i}")).collect()
    }

    /// Clone a source prim at user-specified destination paths.
    ///
    /// Clones are placed at the positions/orientations in `options`; the
    /// source prim's transform ops are normalized to
    /// `translate * orient * scale` first, as in the legacy implementation.
    ///
    /// Errors if the source path is invalid, the source prim does not exist,
    /// or the dimensions of positions/orientations do not match `prim_paths`.
    pub fn clone<S: SceneStage + ?Sized>(
        &mut self,
        stage: &mut S,
        source_prim_path: &str,
        prim_paths: &[String],
        options: &CloneOptions,
    ) -> Result<(), String> {
        if !is_valid_path_string(source_prim_path) {
            return Err(format!("source_prim_path {source_prim_path:?} is not a valid SdfPath"));
        }
        if let Some(positions) = &options.positions {
            if positions.len() != prim_paths.len() {
                return Err("Dimension mismatch between positions and prim_paths!".to_string());
            }
        }
        if let Some(orientations) = &options.orientations {
            if orientations.len() != prim_paths.len() {
                return Err("Dimension mismatch between orientations and prim_paths!".to_string());
            }
        }
        if !stage.prim_exists(source_prim_path) {
            return Err("Source prim does not exist".to_string());
        }

        // Normalize the source xform ops to translate/orient/scale, keeping
        // current values (legacy removes rotate*/transform ops and re-authors
        // the three canonical ops).
        let current_translation = stage
            .attribute(source_prim_path, "xformOp:translate")
            .and_then(|v| v.as_vec3d())
            .unwrap_or([0.0, 0.0, 0.0]);
        let orient_value = stage.attribute(source_prim_path, "xformOp:orient");
        let orient_is_float = matches!(orient_value, Some(Value::Quatf(_)));
        let current_orientation = orient_value
            .and_then(|v| v.as_quatd())
            .unwrap_or([1.0, 0.0, 0.0, 0.0]);
        let current_scale = stage
            .attribute(source_prim_path, "xformOp:scale")
            .and_then(|v| v.as_vec3d())
            .unwrap_or([1.0, 1.0, 1.0]);

        for op in [
            "xformOp:rotateX",
            "xformOp:rotateXZY",
            "xformOp:rotateY",
            "xformOp:rotateYXZ",
            "xformOp:rotateYZX",
            "xformOp:rotateZ",
            "xformOp:rotateZYX",
            "xformOp:rotateZXY",
            "xformOp:rotateXYZ",
            "xformOp:transform",
        ] {
            stage.remove_attribute(source_prim_path, op)?;
        }
        let orient = |q: [f64; 4]| {
            if orient_is_float {
                Value::Quatf([q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32])
            } else {
                Value::Quatd(q)
            }
        };
        let xform_op_order = Value::TokenArray(vec![
            "xformOp:translate".to_string(),
            "xformOp:orient".to_string(),
            "xformOp:scale".to_string(),
        ]);
        stage.set_attribute(
            source_prim_path,
            "xformOp:translate",
            Value::Vec3d(current_translation),
        )?;
        stage.set_attribute(source_prim_path, "xformOp:orient", orient(current_orientation))?;
        stage.set_attribute(source_prim_path, "xformOp:scale", Value::Vec3d(current_scale))?;
        stage.set_attribute(source_prim_path, "xformOpOrder", xform_op_order.clone())?;

        for (i, prim_path) in prim_paths.iter().enumerate() {
            let translation = options
                .positions
                .as_ref()
                .map_or(current_translation, |positions| positions[i]);
            let orientation = options
                .orientations
                .as_ref()
                .map_or(current_orientation, |orientations| orientations[i]);

            if prim_path == source_prim_path {
                // Overwrite the source's transform to the values specified
                stage.set_attribute(prim_path, "xformOp:translate", Value::Vec3d(translation))?;
                stage.set_attribute(prim_path, "xformOp:orient", orient(orientation))?;
                continue;
            }

            if options.copy_from_source {
                stage.copy_spec(source_prim_path, prim_path)?;
            } else {
                stage.define_prim(prim_path, "")?;
                stage.add_inherit(prim_path, source_prim_path)?;
            }

            // Author the same orient precision as the source (legacy
            // precision handling): copies carry the source's Quatf/Quatd
            // spec and inherit clones compose the source's attribute type.
            stage.set_attribute(prim_path, "xformOp:translate", Value::Vec3d(translation))?;
            stage.set_attribute(prim_path, "xformOp:orient", orient(orientation))?;
            stage.set_attribute(prim_path, "xformOp:scale", Value::Vec3d(current_scale))?;
            stage.set_attribute(prim_path, "xformOpOrder", xform_op_order.clone())?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use isaacsim_scene::Stage;

    /// Port of legacy `tests/test_cloner.py::test_simple_cloner`.
    #[test]
    fn test_simple_cloner() {
        let mut stage = Stage::new();
        stage.define_prim("/World/Cube_0", "Cube").unwrap();

        let mut cloner = Cloner::new();
        let target_paths = cloner.generate_paths("/World/Cube", 4);
        assert_eq!(
            target_paths,
            ["/World/Cube_0", "/World/Cube_1", "/World/Cube_2", "/World/Cube_3"]
        );

        let positions = vec![[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [6.0, 0.0, 0.0], [9.0, 0.0, 0.0]];
        cloner
            .clone(
                &mut stage,
                "/World/Cube_0",
                &target_paths,
                &CloneOptions {
                    positions: Some(positions.clone()),
                    ..Default::default()
                },
            )
            .unwrap();

        for (i, position) in positions.iter().enumerate() {
            let path = format!("/World/Cube_{i}");
            assert!(stage.prim_exists(&path));
            assert_eq!(stage.composed_type_name(&path).unwrap(), "Cube");
            assert_eq!(
                stage.resolve_attribute(&path, "xformOp:translate"),
                Some(Value::Vec3d(*position))
            );
        }
    }

    /// Port of legacy `test_clone_rejects_invalid_source_prim_path`.
    #[test]
    fn test_clone_rejects_invalid_source_prim_path() {
        let mut stage = Stage::new();
        let mut cloner = Cloner::new();
        let err = cloner
            .clone(&mut stage, "not a valid path!!!", &[], &CloneOptions::default())
            .unwrap_err();
        assert!(err.contains("valid SdfPath"), "unexpected error: {err}");
    }

    /// Port of legacy `test_clone_restores_change_listener_when_validation_raises`
    /// (the validation part: mismatched positions fail before any mutation).
    #[test]
    fn test_clone_rejects_dimension_mismatch() {
        let mut stage = Stage::new();
        stage.define_prim("/World/Cube_0", "Cube").unwrap();
        let mut cloner = Cloner::new();
        let err = cloner
            .clone(
                &mut stage,
                "/World/Cube_0",
                &["/World/Cube_0".to_string()],
                &CloneOptions {
                    positions: Some(vec![[0.0; 3], [1.0; 3]]),
                    ..Default::default()
                },
            )
            .unwrap_err();
        assert!(err.contains("positions"), "unexpected error: {err}");
    }

    #[test]
    fn test_clone_missing_source_fails() {
        let mut stage = Stage::new();
        let mut cloner = Cloner::new();
        let err = cloner
            .clone(&mut stage, "/World/Missing", &[], &CloneOptions::default())
            .unwrap_err();
        assert_eq!(err, "Source prim does not exist");
    }

    /// Port of the legacy quatf cloner test: a float-precision source orient
    /// is preserved on copied clones.
    #[test]
    fn test_quatf_precision_preserved_on_copy() {
        let mut stage = Stage::new();
        stage.define_prim("/World/Xform_0", "Xform").unwrap();
        stage
            .set_attribute("/World/Xform_0", "xformOp:orient", Value::Quatf([1.0, 0.0, 0.0, 0.0]))
            .unwrap();

        let mut cloner = Cloner::new();
        let target_paths = cloner.generate_paths("/World/Xform", 2);
        cloner
            .clone(
                &mut stage,
                "/World/Xform_0",
                &target_paths,
                &CloneOptions {
                    positions: Some(vec![[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]]),
                    copy_from_source: true,
                    ..Default::default()
                },
            )
            .unwrap();

        assert!(matches!(
            stage.get_attribute("/World/Xform_1", "xformOp:orient"),
            Some(Value::Quatf(_))
        ));
        assert_eq!(
            stage.resolve_attribute("/World/Xform_1", "xformOp:translate"),
            Some(Value::Vec3d([3.0, 0.0, 0.0]))
        );
    }

    /// Port of legacy `test_grid_cloner_inherit_addition` /
    /// `test_grid_cloner_copy_addition` semantics: additions to the source
    /// appear in inherit clones but not copy clones.
    #[test]
    fn test_inherit_vs_copy_addition() {
        for copy_from_source in [false, true] {
            let mut stage = Stage::new();
            stage.define_prim("/World/envs/env_0", "Xform").unwrap();
            stage.define_prim("/World/envs/env_0/Ant", "Xform").unwrap();

            let mut cloner = Cloner::new();
            cloner.define_base_env(&mut stage, "/World/envs").unwrap();
            let target_paths = cloner.generate_paths("/World/envs/env", 4);
            cloner
                .clone(
                    &mut stage,
                    "/World/envs/env_0",
                    &target_paths,
                    &CloneOptions {
                        copy_from_source,
                        ..Default::default()
                    },
                )
                .unwrap();

            for i in 0..4 {
                assert!(stage.prim_exists(&format!("/World/envs/env_{i}")));
                assert!(stage.prim_exists(&format!("/World/envs/env_{i}/Ant")));
            }

            // Add prims after cloning (legacy adds Cube to env_0, Sphere to env_1)
            stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
            stage.define_prim("/World/envs/env_1/Sphere", "Sphere").unwrap();
            assert!(stage.prim_exists("/World/envs/env_0/Cube"));
            assert!(!stage.prim_exists("/World/envs/env_0/Sphere"));
            assert!(stage.prim_exists("/World/envs/env_1/Sphere"));
            // Inherit clones see the source addition; copies do not
            assert_eq!(stage.prim_exists("/World/envs/env_1/Cube"), !copy_from_source);
        }
    }
}
