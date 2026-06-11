// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! The ported cloner running on a real OpenUSD stage via FFI.
//!
//! Mirrors the legacy `test_cloner.py` cases previously verified against the
//! in-memory backend, now with real USD doing the composition.

#![cfg(feature = "openusd")]

use isaacsim_core_cloner::GridCloner;
use isaacsim_scene::{SceneStage, Stage, UpAxis, Value};
use isaacsim_usd::UsdStage;

/// Port of legacy `test_grid_cloner`, on real USD.
#[test]
fn test_grid_cloner_on_real_usd() {
    let mut stage = UsdStage::create_in_memory().unwrap();
    stage.set_up_axis(UpAxis::Z);
    stage.define_prim("/World/Cube_0", "Cube").unwrap();

    let mut cloner = GridCloner::new(3.0, None);
    let target_paths = cloner.generate_paths("/World/Cube", 4);
    let positions = cloner
        .clone(&mut stage, "/World/Cube_0", &target_paths, None, None, false)
        .unwrap();

    let target_translations = [
        [1.5, -1.5, 0.0],
        [1.5, 1.5, 0.0],
        [-1.5, -1.5, 0.0],
        [-1.5, 1.5, 0.0],
    ];
    assert_eq!(positions, target_translations);
    for (i, translation) in target_translations.iter().enumerate() {
        let path = format!("/World/Cube_{i}");
        assert!(stage.prim_exists(&path));
        // Real USD composes the clone's type through the inherit arc
        assert_eq!(stage.type_name(&path).unwrap(), "Cube");
        assert_eq!(
            stage.attribute(&path, "xformOp:translate"),
            Some(Value::Vec3d(*translation))
        );
        assert_eq!(
            stage.attribute(&path, "xformOpOrder"),
            Some(Value::TokenArray(vec![
                "xformOp:translate".into(),
                "xformOp:orient".into(),
                "xformOp:scale".into(),
            ]))
        );
    }
}

/// Port of legacy `test_grid_cloner_inherit_addition` / `copy_addition`:
/// additions to the source appear in inherit clones but not copy clones —
/// with real USD composition.
#[test]
fn test_inherit_vs_copy_addition_on_real_usd() {
    for copy_from_source in [false, true] {
        let mut stage = UsdStage::create_in_memory().unwrap();
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        stage.define_prim("/World/envs/env_0/Ant", "Xform").unwrap();

        let mut cloner = GridCloner::new(3.0, None);
        cloner.define_base_env(&mut stage, "/World/envs").unwrap();
        let target_paths = cloner.generate_paths("/World/envs/env", 4);
        cloner
            .clone(
                &mut stage,
                "/World/envs/env_0",
                &target_paths,
                None,
                None,
                copy_from_source,
            )
            .unwrap();

        for i in 0..4 {
            assert!(stage.prim_exists(&format!("/World/envs/env_{i}")));
            assert!(stage.prim_exists(&format!("/World/envs/env_{i}/Ant")));
        }

        stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
        stage.define_prim("/World/envs/env_1/Sphere", "Sphere").unwrap();
        assert!(stage.prim_exists("/World/envs/env_0/Cube"));
        assert!(!stage.prim_exists("/World/envs/env_0/Sphere"));
        assert!(stage.prim_exists("/World/envs/env_1/Sphere"));
        assert_eq!(
            stage.prim_exists("/World/envs/env_1/Cube"),
            !copy_from_source,
            "copy_from_source={copy_from_source}"
        );
    }
}

/// The same clone operation on both backends produces equivalent stages,
/// checked through the real-USD export and the in-memory subset reader.
#[test]
fn test_backends_agree() {
    let run = |stage: &mut dyn SceneStage| -> Vec<[f64; 3]> {
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
        stage
            .set_attribute("/World/envs/env_0/Cube", "size", Value::Double(0.5))
            .unwrap();
        let mut cloner = GridCloner::new(2.0, None);
        let target_paths = cloner.generate_paths("/World/envs/env", 9);
        cloner
            .clone(stage, "/World/envs/env_0", &target_paths, None, None, false)
            .unwrap()
    };

    let mut usd = UsdStage::create_in_memory().unwrap();
    usd.set_up_axis(UpAxis::Z);
    let mut memory = Stage::new();
    let positions_usd = run(&mut usd);
    let positions_memory = run(&mut memory);
    assert_eq!(positions_usd, positions_memory);

    for i in 0..9 {
        let path = format!("/World/envs/env_{i}");
        assert_eq!(
            usd.attribute(&path, "xformOp:translate"),
            memory.attribute(&path, "xformOp:translate"),
            "translate mismatch at {path}"
        );
        assert_eq!(usd.type_name(&path), memory.type_name(&path));
        assert_eq!(
            usd.attribute(&format!("{path}/Cube"), "size"),
            memory.attribute(&format!("{path}/Cube"), "size")
        );
    }

    // Real USD's export parses back through the in-memory subset reader
    let exported = usd.export_to_string().unwrap();
    let parsed = Stage::from_usda(&exported).unwrap();
    assert!(parsed.prim_exists("/World/envs/env_8"));
    assert_eq!(
        parsed.resolve_attribute("/World/envs/env_8", "xformOp:translate"),
        memory.attribute("/World/envs/env_8", "xformOp:translate")
    );
}

/// Object-safety check: the cloner surface works through `&mut dyn SceneStage`.
#[test]
fn test_scene_stage_is_object_safe() {
    let mut usd = UsdStage::create_in_memory().unwrap();
    let stage: &mut dyn SceneStage = &mut usd;
    stage.define_prim("/World", "Xform").unwrap();
    stage
        .set_attribute("/World", "kind", Value::Token("group".into()))
        .unwrap();
    assert_eq!(stage.attribute("/World", "kind"), Some(Value::Token("group".into())));
    stage.remove_attribute("/World", "kind").unwrap();
    assert_eq!(stage.attribute("/World", "kind"), None);
}
