// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! End-to-end test: grid-clone a prim, export the stage as `.usda`,
//! and round-trip it through the subset reader.
//!
//! The exported file is also written to the target tmp dir so it can be
//! validated against real USD (`pxr.Usd`) out of band.

use isaacsim_core_cloner::GridCloner;
use isaacsim_scene::{Stage, Value};

#[test]
fn test_grid_clone_export_round_trip() {
    let mut stage = Stage::new();
    stage.define_prim("/World/envs/env_0", "Xform").unwrap();
    stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
    stage
        .set_attribute("/World/envs/env_0/Cube", "size", Value::Double(0.5))
        .unwrap();

    let mut cloner = GridCloner::new(3.0, None);
    cloner.define_base_env(&mut stage, "/World/envs").unwrap();
    let target_paths = cloner.generate_paths("/World/envs/env", 4);
    let positions = cloner
        .clone(&mut stage, "/World/envs/env_0", &target_paths, None, None, false)
        .unwrap();
    assert_eq!(positions.len(), 4);

    let usda = stage.to_usda();
    let parsed = Stage::from_usda(&usda).unwrap();

    for (i, position) in positions.iter().enumerate() {
        let path = format!("/World/envs/env_{i}");
        assert!(parsed.prim_exists(&path));
        assert!(parsed.prim_exists(&format!("{path}/Cube")));
        assert_eq!(
            parsed.resolve_attribute(&path, "xformOp:translate"),
            Some(Value::Vec3d(*position))
        );
        assert_eq!(
            parsed.resolve_attribute(&format!("{path}/Cube"), "size"),
            Some(Value::Double(0.5))
        );
    }

    // Persist for out-of-band validation with real USD
    let out = std::env::temp_dir().join("isaacsim_cloner_test.usda");
    stage.save_usda(&out).unwrap();
}
