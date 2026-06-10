// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! In-memory stage: a prim hierarchy with typed attributes and inherit
//! composition.
//!
//! Composition is the subset the ported logic layer relies on: a prim spec
//! can inherit other prims (`Sdf.InheritPathList`), and composed queries
//! ([`Stage::prim_exists`], [`Stage::composed_type_name`],
//! [`Stage::resolve_attribute`]) resolve through the inherit chain with
//! local opinions winning, matching USD strength ordering.

use std::collections::BTreeMap;

use crate::path::{is_valid_path_string, parent_path};
use crate::value::Value;

/// Stage up axis (`UsdGeom.GetStageUpAxis`).
///
/// Defaults to [`UpAxis::Z`], the Isaac Sim stage convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UpAxis {
    Y,
    #[default]
    Z,
}

/// A prim spec: type name, attributes, and inherit list.
#[derive(Debug, Clone, Default)]
pub struct Prim {
    /// Prim type name (e.g. `"Cube"`, `"Xform"`, `"Scope"`); empty for
    /// typeless specs created as ancestors or clone targets.
    pub type_name: String,
    /// Authored attributes by name.
    pub attributes: BTreeMap<String, Value>,
    /// Paths this prim inherits from, strongest first.
    pub inherits: Vec<String>,
}

/// In-memory USD-like stage.
#[derive(Debug, Clone, Default)]
pub struct Stage {
    up_axis: UpAxis,
    prims: BTreeMap<String, Prim>,
}

impl Stage {
    /// Create an empty stage with the default (Z) up axis.
    pub fn new() -> Self {
        Self::default()
    }

    /// Stage up axis.
    pub fn up_axis(&self) -> UpAxis {
        self.up_axis
    }

    /// Set the stage up axis (`UsdGeom.SetStageUpAxis`).
    pub fn set_up_axis(&mut self, up_axis: UpAxis) {
        self.up_axis = up_axis;
    }

    /// Define a prim at `path` with the given type name, creating missing
    /// ancestors as typeless specs (`Sdf.CreatePrimInLayer` behavior). If the
    /// prim already exists, a non-empty `type_name` overrides its type.
    ///
    /// Returns an error for invalid paths or the pseudo-root.
    pub fn define_prim(&mut self, path: &str, type_name: &str) -> Result<&mut Prim, String> {
        if path == "/" {
            return Err("cannot define a prim at the pseudo-root '/'".to_string());
        }
        if !is_valid_path_string(path) {
            return Err(format!("{path:?} is not a valid SdfPath"));
        }
        let mut ancestor = parent_path(path);
        while let Some(p) = ancestor {
            if p == "/" {
                break;
            }
            self.prims.entry(p.to_string()).or_default();
            ancestor = parent_path(p);
        }
        let prim = self.prims.entry(path.to_string()).or_default();
        if !type_name.is_empty() {
            prim.type_name = type_name.to_string();
        }
        Ok(prim)
    }

    /// The prim spec authored at `path`, if any (no composition).
    pub fn get_prim(&self, path: &str) -> Option<&Prim> {
        self.prims.get(path)
    }

    /// Mutable access to the prim spec authored at `path`.
    pub fn get_prim_mut(&mut self, path: &str) -> Option<&mut Prim> {
        self.prims.get_mut(path)
    }

    /// Set an attribute on the prim at `path`. Errors if the prim spec does
    /// not exist.
    pub fn set_attribute(&mut self, path: &str, name: &str, value: Value) -> Result<(), String> {
        let prim = self
            .prims
            .get_mut(path)
            .ok_or_else(|| format!("prim {path} does not exist"))?;
        prim.attributes.insert(name.to_string(), value);
        Ok(())
    }

    /// The attribute authored directly on the prim at `path` (no composition).
    pub fn get_attribute(&self, path: &str, name: &str) -> Option<&Value> {
        self.prims.get(path)?.attributes.get(name)
    }

    /// Whether a prim exists at `path` in the composed stage: either authored
    /// directly, or brought in through an ancestor's inherit
    /// (`Usd.Prim.IsValid` on the composed stage).
    pub fn prim_exists(&self, path: &str) -> bool {
        self.compose(path, &mut |_| true).unwrap_or(false)
    }

    /// The composed type name of the prim at `path`: the first non-empty type
    /// found following local-then-inherits strength ordering.
    pub fn composed_type_name(&self, path: &str) -> Option<String> {
        self.compose(path, &mut |prim| {
            if prim.type_name.is_empty() {
                None
            } else {
                Some(prim.type_name.clone())
            }
        })
        .flatten()
        .or_else(|| {
            if self.prim_exists(path) {
                Some(String::new())
            } else {
                None
            }
        })
    }

    /// The composed value of attribute `name` on the prim at `path`: the
    /// strongest opinion following local-then-inherits ordering.
    pub fn resolve_attribute(&self, path: &str, name: &str) -> Option<Value> {
        self.compose(path, &mut |prim| prim.attributes.get(name).cloned())
            .flatten()
    }

    /// All authored prim specs in path order.
    pub fn prims(&self) -> impl Iterator<Item = (&str, &Prim)> {
        self.prims.iter().map(|(p, prim)| (p.as_str(), prim))
    }

    /// Direct children of the prim at `path` (authored specs only).
    pub fn children(&self, path: &str) -> Vec<&str> {
        let prefix = if path == "/" {
            "/".to_string()
        } else {
            format!("{path}/")
        };
        self.prims
            .range(prefix.clone()..)
            .take_while(|(p, _)| p.starts_with(&prefix))
            .filter(|(p, _)| !p[prefix.len()..].contains('/'))
            .map(|(p, _)| p.as_str())
            .collect()
    }

    /// Deep-copy the prim subtree at `source` to `dest`
    /// (`Sdf.CopySpec`): attributes, inherit lists, and all descendants.
    ///
    /// Errors if the source spec does not exist or either path is invalid.
    pub fn copy_spec(&mut self, source: &str, dest: &str) -> Result<(), String> {
        if !self.prims.contains_key(source) {
            return Err(format!("source spec {source} does not exist"));
        }
        if !is_valid_path_string(dest) || dest == "/" {
            return Err(format!("{dest:?} is not a valid SdfPath"));
        }
        let child_prefix = format!("{source}/");
        let subtree: Vec<(String, Prim)> = self
            .prims
            .range(source.to_string()..)
            .take_while(|(p, _)| *p == source || p.starts_with(&child_prefix))
            .map(|(p, prim)| (format!("{dest}{}", &p[source.len()..]), prim.clone()))
            .collect();
        self.define_prim(dest, "")?;
        for (path, prim) in subtree {
            self.prims.insert(path, prim);
        }
        Ok(())
    }

    /// Resolve `path` against local specs and ancestor inherits, applying `f`
    /// to the first prim spec found in strength order. Returns `None` if no
    /// spec contributes to `path` or `f` returns `None` everywhere.
    fn compose<T>(&self, path: &str, f: &mut dyn FnMut(&Prim) -> T) -> Option<T>
    where
        T: IntoComposed,
    {
        self.compose_guarded(path, f, 0)
    }

    fn compose_guarded<T>(&self, path: &str, f: &mut dyn FnMut(&Prim) -> T, depth: usize) -> Option<T>
    where
        T: IntoComposed,
    {
        // Inherit chains are authored data; bail out on pathological cycles.
        const MAX_DEPTH: usize = 64;
        if depth > MAX_DEPTH {
            return None;
        }
        // Local opinion is strongest.
        if let Some(prim) = self.prims.get(path) {
            let result = f(prim);
            if result.is_composed() {
                return Some(result);
            }
        }
        // Then opinions brought in by inherits on this prim or any ancestor,
        // nearest ancestor first.
        let mut current = Some(path);
        while let Some(p) = current {
            if let Some(prim) = self.prims.get(p) {
                let suffix = &path[p.len()..];
                for inherited in &prim.inherits {
                    let candidate = format!("{inherited}{suffix}");
                    if let Some(result) = self.compose_guarded(&candidate, f, depth + 1) {
                        if result.is_composed() {
                            return Some(result);
                        }
                    }
                }
            }
            current = parent_path(p).filter(|p| *p != "/");
        }
        None
    }
}

/// Helper for [`Stage::compose`]: distinguishes "spec found but no opinion"
/// (`Option::None`) from a real composed result, while letting boolean
/// existence queries short-circuit.
trait IntoComposed {
    fn is_composed(&self) -> bool;
}

impl IntoComposed for bool {
    fn is_composed(&self) -> bool {
        *self
    }
}

impl<T> IntoComposed for Option<T> {
    fn is_composed(&self) -> bool {
        self.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_define_prim_creates_ancestors() {
        let mut stage = Stage::new();
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        assert!(stage.get_prim("/World").is_some());
        assert!(stage.get_prim("/World/envs").is_some());
        assert_eq!(stage.get_prim("/World/envs/env_0").unwrap().type_name, "Xform");
        // Ancestors are typeless specs
        assert_eq!(stage.get_prim("/World").unwrap().type_name, "");

        assert!(stage.define_prim("not a path", "Cube").is_err());
        assert!(stage.define_prim("/", "Cube").is_err());
    }

    #[test]
    fn test_attributes() {
        let mut stage = Stage::new();
        stage.define_prim("/World/Cube", "Cube").unwrap();
        stage
            .set_attribute("/World/Cube", "xformOp:translate", Value::Vec3d([1.0, 2.0, 3.0]))
            .unwrap();
        assert_eq!(
            stage.get_attribute("/World/Cube", "xformOp:translate"),
            Some(&Value::Vec3d([1.0, 2.0, 3.0]))
        );
        assert!(stage.set_attribute("/Missing", "a", Value::Bool(true)).is_err());
    }

    #[test]
    fn test_inherit_composition() {
        let mut stage = Stage::new();
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        stage.define_prim("/World/envs/env_0/Ant", "Xform").unwrap();
        stage
            .set_attribute("/World/envs/env_0/Ant", "mass", Value::Double(2.0))
            .unwrap();

        // env_1 inherits env_0
        stage.define_prim("/World/envs/env_1", "").unwrap();
        stage
            .get_prim_mut("/World/envs/env_1")
            .unwrap()
            .inherits
            .push("/World/envs/env_0".to_string());

        // Composed type and descendants come from the inherited source
        assert_eq!(stage.composed_type_name("/World/envs/env_1").unwrap(), "Xform");
        assert!(stage.prim_exists("/World/envs/env_1/Ant"));
        assert_eq!(
            stage.resolve_attribute("/World/envs/env_1/Ant", "mass"),
            Some(Value::Double(2.0))
        );

        // Changes to the source are reflected in the inheriting clone
        stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
        assert!(stage.prim_exists("/World/envs/env_1/Cube"));

        // Local opinions win over inherited ones
        stage.define_prim("/World/envs/env_1/Ant", "").unwrap();
        stage
            .set_attribute("/World/envs/env_1/Ant", "mass", Value::Double(5.0))
            .unwrap();
        assert_eq!(
            stage.resolve_attribute("/World/envs/env_1/Ant", "mass"),
            Some(Value::Double(5.0))
        );

        // Non-existent paths stay non-existent
        assert!(!stage.prim_exists("/World/envs/env_1/Sphere"));
        assert!(!stage.prim_exists("/World/other"));
    }

    #[test]
    fn test_copy_spec() {
        let mut stage = Stage::new();
        stage.define_prim("/World/envs/env_0", "Xform").unwrap();
        stage.define_prim("/World/envs/env_0/Ant", "Xform").unwrap();
        stage
            .set_attribute("/World/envs/env_0/Ant", "mass", Value::Double(2.0))
            .unwrap();

        stage.copy_spec("/World/envs/env_0", "/World/envs/env_1").unwrap();
        assert_eq!(stage.composed_type_name("/World/envs/env_1").unwrap(), "Xform");
        assert_eq!(
            stage.resolve_attribute("/World/envs/env_1/Ant", "mass"),
            Some(Value::Double(2.0))
        );

        // Copies are independent: changes to the source are not reflected
        stage.define_prim("/World/envs/env_0/Cube", "Cube").unwrap();
        assert!(!stage.prim_exists("/World/envs/env_1/Cube"));

        assert!(stage.copy_spec("/Missing", "/World/envs/env_2").is_err());
    }

    #[test]
    fn test_children() {
        let mut stage = Stage::new();
        stage.define_prim("/World/A", "Xform").unwrap();
        stage.define_prim("/World/B", "Xform").unwrap();
        stage.define_prim("/World/B/C", "Xform").unwrap();
        assert_eq!(stage.children("/World"), vec!["/World/A", "/World/B"]);
        assert_eq!(stage.children("/"), vec!["/World"]);
        assert!(stage.children("/World/A").is_empty());
    }

    #[test]
    fn test_inherit_cycle_terminates() {
        let mut stage = Stage::new();
        stage.define_prim("/a", "").unwrap();
        stage.define_prim("/b", "").unwrap();
        stage.get_prim_mut("/a").unwrap().inherits.push("/b".to_string());
        stage.get_prim_mut("/b").unwrap().inherits.push("/a".to_string());
        // No opinion anywhere; must terminate rather than recurse forever
        assert!(stage.resolve_attribute("/a", "missing").is_none());
        assert!(!stage.prim_exists("/a/child"));
    }
}
