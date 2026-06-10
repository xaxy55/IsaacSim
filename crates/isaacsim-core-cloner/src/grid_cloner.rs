// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Grid cloner module.
//!
//! Port of `python/impl/grid_cloner.py` from the legacy `isaacsim.core.cloner`
//! extension.

use isaacsim_scene::{SceneStage, UpAxis};

use crate::cloner::{CloneOptions, Cloner};

/// A specialized [`Cloner`] that automatically generates clones in a grid
/// pattern.
///
/// `num_per_row` defaults to `sqrt(num_clones)` when `None` (legacy `-1`).
///
/// Note: deliberately not `Clone` — the legacy-named [`Self::clone`] prim
/// duplication method would otherwise be shadowed by `Clone::clone`.
#[derive(Debug)]
pub struct GridCloner {
    cloner: Cloner,
    spacing: f64,
    num_per_row: Option<usize>,
    cached_positions: Option<Vec<[f64; 3]>>,
    cached_orientations: Option<Vec<[f64; 4]>>,
    cached_num_clones: Option<usize>,
}

impl GridCloner {
    /// Create a grid cloner with the given spacing between clones.
    pub fn new(spacing: f64, num_per_row: Option<usize>) -> Self {
        Self {
            cloner: Cloner::new(),
            spacing,
            num_per_row,
            cached_positions: None,
            cached_orientations: None,
            cached_num_clones: None,
        }
    }

    /// Access the underlying [`Cloner`] (e.g. for `define_base_env`).
    pub fn cloner_mut(&mut self) -> &mut Cloner {
        &mut self.cloner
    }

    /// Create a `Scope` prim at `base_env_path` (see [`Cloner::define_base_env`]).
    pub fn define_base_env<S: SceneStage + ?Sized>(
        &mut self,
        stage: &mut S,
        base_env_path: &str,
    ) -> Result<(), String> {
        self.cloner.define_base_env(stage, base_env_path)
    }

    /// Generate `num_paths` paths in the format `{root_path}_{i}`
    /// (see [`Cloner::generate_paths`]).
    pub fn generate_paths(&mut self, root_path: &str, num_paths: usize) -> Vec<String> {
        self.cloner.generate_paths(root_path, num_paths)
    }

    /// Compute the positions and orientations of clones in a grid.
    ///
    /// `position_offsets` / `orientation_offsets` (quaternions `[w, x, y, z]`)
    /// are applied per clone on top of the computed grid transform. Plain
    /// grid transforms (no offsets) are cached per clone count, as in the
    /// legacy implementation.
    ///
    /// Errors if an offset array length does not match `num_clones` or
    /// `num_per_row` is zero.
    #[allow(clippy::type_complexity)]
    pub fn get_clone_transforms<S: SceneStage + ?Sized>(
        &mut self,
        stage: &S,
        num_clones: usize,
        position_offsets: Option<&[[f64; 3]]>,
        orientation_offsets: Option<&[[f64; 4]]>,
    ) -> Result<(Vec<[f64; 3]>, Vec<[f64; 4]>), String> {
        if let Some(offsets) = position_offsets {
            if offsets.len() != num_clones {
                return Err("Dimension mismatch between position_offsets and prim_paths!".to_string());
            }
        }
        if let Some(offsets) = orientation_offsets {
            if offsets.len() != num_clones {
                return Err("Dimension mismatch between orientation_offsets and prim_paths!".to_string());
            }
        }

        let use_cache = position_offsets.is_none() && orientation_offsets.is_none();
        if use_cache && self.cached_num_clones == Some(num_clones) {
            if let (Some(positions), Some(orientations)) =
                (&self.cached_positions, &self.cached_orientations)
            {
                return Ok((positions.clone(), orientations.clone()));
            }
        }

        if num_clones == 0 {
            if use_cache {
                self.cached_positions = Some(Vec::new());
                self.cached_orientations = Some(Vec::new());
                self.cached_num_clones = Some(0);
            }
            return Ok((Vec::new(), Vec::new()));
        }

        let num_per_row = match self.num_per_row {
            Some(n) => n,
            None => (num_clones as f64).sqrt() as usize,
        };
        if num_per_row == 0 {
            return Err("num_per_row must be positive".to_string());
        }
        let num_rows = num_clones.div_ceil(num_per_row);
        let num_cols = num_clones.div_ceil(num_rows);

        let row_offset = 0.5 * self.spacing * (num_rows - 1) as f64;
        let col_offset = 0.5 * self.spacing * (num_cols - 1) as f64;

        let mut positions = Vec::with_capacity(num_clones);
        let mut orientations = Vec::with_capacity(num_clones);
        for i in 0..num_clones {
            let row = i / num_cols;
            let col = i % num_cols;
            let x = row_offset - row as f64 * self.spacing;
            let y = col as f64 * self.spacing - col_offset;

            let position = match stage.up_axis() {
                UpAxis::Z => [x, y, 0.0],
                UpAxis::Y => [x, 0.0, y],
            };
            let translation = match position_offsets {
                Some(offsets) => [
                    offsets[i][0] + position[0],
                    offsets[i][1] + position[1],
                    offsets[i][2] + position[2],
                ],
                None => position,
            };
            // offset * identity = offset
            let orientation = match orientation_offsets {
                Some(offsets) => offsets[i],
                None => [1.0, 0.0, 0.0, 0.0],
            };

            positions.push(translation);
            orientations.push(orientation);
        }

        if use_cache {
            self.cached_positions = Some(positions.clone());
            self.cached_orientations = Some(orientations.clone());
            self.cached_num_clones = Some(num_clones);
        }
        Ok((positions, orientations))
    }

    /// Create clones in a grid pattern with automatically computed positions.
    ///
    /// Returns the computed positions of all clones.
    pub fn clone<S: SceneStage + ?Sized>(
        &mut self,
        stage: &mut S,
        source_prim_path: &str,
        prim_paths: &[String],
        position_offsets: Option<&[[f64; 3]]>,
        orientation_offsets: Option<&[[f64; 4]]>,
        copy_from_source: bool,
    ) -> Result<Vec<[f64; 3]>, String> {
        let (positions, orientations) =
            self.get_clone_transforms(stage, prim_paths.len(), position_offsets, orientation_offsets)?;
        self.cloner.clone(
            stage,
            source_prim_path,
            prim_paths,
            &CloneOptions {
                positions: Some(positions.clone()),
                orientations: Some(orientations),
                copy_from_source,
            },
        )?;
        Ok(positions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use isaacsim_scene::{Stage, Value};

    /// Port of legacy `tests/test_cloner.py::test_grid_cloner`.
    #[test]
    fn test_grid_cloner() {
        let mut stage = Stage::new();
        stage.define_prim("/World/Cube_0", "Cube").unwrap();

        let mut cloner = GridCloner::new(3.0, None);
        let target_paths = cloner.generate_paths("/World/Cube", 4);
        cloner
            .clone(&mut stage, "/World/Cube_0", &target_paths, None, None, false)
            .unwrap();

        let target_translations = [
            [1.5, -1.5, 0.0],
            [1.5, 1.5, 0.0],
            [-1.5, -1.5, 0.0],
            [-1.5, 1.5, 0.0],
        ];
        for (i, translation) in target_translations.iter().enumerate() {
            let path = format!("/World/Cube_{i}");
            assert!(stage.prim_exists(&path));
            assert_eq!(stage.composed_type_name(&path).unwrap(), "Cube");
            assert_eq!(
                stage.resolve_attribute(&path, "xformOp:translate"),
                Some(Value::Vec3d(*translation))
            );
        }
    }

    /// Port of legacy `test_grid_cloner_recomputes_transforms_when_count_changes`.
    #[test]
    fn test_grid_cloner_recomputes_transforms_when_count_changes() {
        let mut stage = Stage::new();
        stage.set_up_axis(UpAxis::Z);

        let mut cloner = GridCloner::new(2.0, None);
        let (positions_4, _) = cloner.get_clone_transforms(&stage, 4, None, None).unwrap();
        let (positions_9, _) = cloner.get_clone_transforms(&stage, 9, None, None).unwrap();

        assert_eq!(positions_4.len(), 4);
        assert_eq!(positions_9.len(), 9);
        assert_eq!(positions_9[0], [2.0, -2.0, 0.0]);
        assert_eq!(positions_9[8], [-2.0, 2.0, 0.0]);
    }

    /// Port of legacy `test_grid_cloner_empty_clone_count_returns_empty_transforms`.
    #[test]
    fn test_grid_cloner_empty_clone_count_returns_empty_transforms() {
        let stage = Stage::new();
        let mut cloner = GridCloner::new(2.0, None);
        let (positions, orientations) = cloner.get_clone_transforms(&stage, 0, None, None).unwrap();
        assert!(positions.is_empty());
        assert!(orientations.is_empty());
    }

    /// Port of legacy `test_grid_cloner_offsets`.
    #[test]
    fn test_grid_cloner_offsets() {
        let mut stage = Stage::new();
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        stage.define_prim("/World/envs/env_0/Ant", "Xform").unwrap();

        let mut cloner = GridCloner::new(3.0, None);
        cloner.define_base_env(&mut stage, "/World/envs").unwrap();
        let target_paths = cloner.generate_paths("/World/envs/env", 100);

        let position_offsets = vec![[0.0, 0.0, 1.0]; 100];
        let orientation_offsets = vec![[0.0, 0.0, 0.0, 1.0]; 100];
        let target_translations = cloner
            .clone(
                &mut stage,
                "/World/envs/env_0",
                &target_paths,
                Some(&position_offsets),
                Some(&orientation_offsets),
                false,
            )
            .unwrap();

        for (i, translation) in target_translations.iter().enumerate() {
            let path = format!("/World/envs/env_{i}");
            assert!(stage.prim_exists(&path));
            assert!(stage.prim_exists(&format!("{path}/Ant")));
            assert_eq!(
                stage.resolve_attribute(&path, "xformOp:translate"),
                Some(Value::Vec3d(*translation))
            );
            assert_eq!(translation[2], 1.0);
            assert_eq!(
                stage.resolve_attribute(&path, "xformOp:orient"),
                Some(Value::Quatd([0.0, 0.0, 0.0, 1.0]))
            );
        }
    }

    /// Grid layout in a Y-up stage places clones in the XZ plane.
    #[test]
    fn test_grid_cloner_y_up() {
        let mut stage = Stage::new();
        stage.set_up_axis(UpAxis::Y);
        let mut cloner = GridCloner::new(3.0, None);
        let (positions, _) = cloner.get_clone_transforms(&stage, 4, None, None).unwrap();
        assert_eq!(positions[0], [1.5, 0.0, -1.5]);
        assert!(positions.iter().all(|p| p[1] == 0.0));
    }

    #[test]
    fn test_offset_dimension_mismatch_fails() {
        let stage = Stage::new();
        let mut cloner = GridCloner::new(2.0, None);
        assert!(cloner
            .get_clone_transforms(&stage, 4, Some(&[[0.0; 3]; 2]), None)
            .is_err());
        assert!(cloner
            .get_clone_transforms(&stage, 4, None, Some(&[[1.0, 0.0, 0.0, 0.0]; 3]))
            .is_err());
    }

    #[test]
    fn test_explicit_num_per_row() {
        let stage = Stage::new();
        // 4 clones, 1 per row: a single column spread along x
        let mut cloner = GridCloner::new(2.0, Some(1));
        let (positions, _) = cloner.get_clone_transforms(&stage, 4, None, None).unwrap();
        assert_eq!(positions[0], [3.0, 0.0, 0.0]);
        assert_eq!(positions[3], [-3.0, 0.0, 0.0]);
    }
}
