// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! [`UsdStage`]: a real OpenUSD stage implementing
//! [`isaacsim_scene::SceneStage`].

use std::ffi::{c_char, c_void, CStr, CString};

use isaacsim_scene::{SceneStage, UpAxis, Value};

// Value type tags shared with src/shim.cpp.
const TAG_BOOL: i32 = 1;
const TAG_INT: i32 = 2;
const TAG_FLOAT: i32 = 3;
const TAG_DOUBLE: i32 = 4;
const TAG_VEC3F: i32 = 5;
const TAG_VEC3D: i32 = 6;
const TAG_QUATF: i32 = 7;
const TAG_QUATD: i32 = 8;
const TAG_TOKEN: i32 = 9;
const TAG_STRING: i32 = 10;
const TAG_TOKEN_ARRAY: i32 = 11;

extern "C" {
    fn isaacsim_usd_stage_create_in_memory() -> *mut c_void;
    fn isaacsim_usd_stage_open(path: *const c_char) -> *mut c_void;
    fn isaacsim_usd_stage_free(handle: *mut c_void);
    fn isaacsim_usd_stage_export(handle: *mut c_void, path: *const c_char) -> bool;
    fn isaacsim_usd_stage_export_string(handle: *mut c_void) -> *mut c_char;
    fn isaacsim_usd_string_free(s: *mut c_char);
    fn isaacsim_usd_up_axis(handle: *mut c_void) -> i32;
    fn isaacsim_usd_set_up_axis(handle: *mut c_void, z_up: i32) -> bool;
    fn isaacsim_usd_define_prim(
        handle: *mut c_void,
        path: *const c_char,
        type_name: *const c_char,
    ) -> bool;
    fn isaacsim_usd_prim_exists(handle: *mut c_void, path: *const c_char) -> bool;
    fn isaacsim_usd_prim_type_name(handle: *mut c_void, path: *const c_char) -> *mut c_char;
    fn isaacsim_usd_add_inherit(
        handle: *mut c_void,
        path: *const c_char,
        source: *const c_char,
    ) -> bool;
    fn isaacsim_usd_copy_spec(
        handle: *mut c_void,
        source: *const c_char,
        dest: *const c_char,
    ) -> bool;
    fn isaacsim_usd_remove_attribute(
        handle: *mut c_void,
        path: *const c_char,
        name: *const c_char,
    ) -> bool;
    fn isaacsim_usd_set_attr_bool(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: bool,
    ) -> bool;
    fn isaacsim_usd_set_attr_int(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: i64,
    ) -> bool;
    fn isaacsim_usd_set_attr_float(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: f32,
    ) -> bool;
    fn isaacsim_usd_set_attr_double(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: f64,
    ) -> bool;
    fn isaacsim_usd_set_attr_vec3f(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: *const f32,
    ) -> bool;
    fn isaacsim_usd_set_attr_vec3d(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: *const f64,
    ) -> bool;
    fn isaacsim_usd_set_attr_quatf(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        q: *const f32,
    ) -> bool;
    fn isaacsim_usd_set_attr_quatd(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        q: *const f64,
    ) -> bool;
    fn isaacsim_usd_set_attr_token(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: *const c_char,
    ) -> bool;
    fn isaacsim_usd_set_attr_string(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        v: *const c_char,
    ) -> bool;
    fn isaacsim_usd_set_attr_token_array(
        h: *mut c_void,
        p: *const c_char,
        n: *const c_char,
        items: *const *const c_char,
        count: i32,
        uniform: bool,
    ) -> bool;
    fn isaacsim_usd_get_attr(
        handle: *mut c_void,
        path: *const c_char,
        name: *const c_char,
        out: *mut f64,
        out_str: *mut *mut c_char,
    ) -> i32;
}

/// USD attribute names declared with `uniform` variability.
const UNIFORM_ATTRIBUTES: &[&str] = &["xformOpOrder"];

/// A real OpenUSD stage.
///
/// Wraps a `UsdStageRefPtr`; the stage is released on drop.
#[derive(Debug)]
pub struct UsdStage {
    handle: *mut c_void,
}

impl UsdStage {
    /// Create an anonymous in-memory stage (`Usd.Stage.CreateInMemory`).
    pub fn create_in_memory() -> Result<Self, String> {
        let handle = unsafe { isaacsim_usd_stage_create_in_memory() };
        if handle.is_null() {
            Err("failed to create an in-memory USD stage".to_string())
        } else {
            Ok(Self { handle })
        }
    }

    /// Open an existing stage from a USD file (`Usd.Stage.Open`).
    pub fn open(path: &str) -> Result<Self, String> {
        let c_path = cstring(path)?;
        let handle = unsafe { isaacsim_usd_stage_open(c_path.as_ptr()) };
        if handle.is_null() {
            Err(format!("failed to open USD stage {path}"))
        } else {
            Ok(Self { handle })
        }
    }

    /// Export the composed-free root layer to a file (`Usd.Stage.Export`).
    pub fn export(&self, path: &str) -> Result<(), String> {
        let c_path = cstring(path)?;
        if unsafe { isaacsim_usd_stage_export(self.handle, c_path.as_ptr()) } {
            Ok(())
        } else {
            Err(format!("failed to export USD stage to {path}"))
        }
    }

    /// Export the stage as `.usda` text (`Usd.Stage.ExportToString`).
    pub fn export_to_string(&self) -> Result<String, String> {
        let ptr = unsafe { isaacsim_usd_stage_export_string(self.handle) };
        if ptr.is_null() {
            return Err("failed to export USD stage to a string".to_string());
        }
        let out = unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned();
        unsafe { isaacsim_usd_string_free(ptr) };
        Ok(out)
    }
}

impl Drop for UsdStage {
    fn drop(&mut self) {
        unsafe { isaacsim_usd_stage_free(self.handle) };
    }
}

fn cstring(s: &str) -> Result<CString, String> {
    CString::new(s).map_err(|_| format!("string contains a NUL byte: {s:?}"))
}

/// Take ownership of a shim-allocated C string.
fn take_string(ptr: *mut c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    let out = unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned();
    unsafe { isaacsim_usd_string_free(ptr) };
    Some(out)
}

impl SceneStage for UsdStage {
    fn up_axis(&self) -> UpAxis {
        if unsafe { isaacsim_usd_up_axis(self.handle) } == 1 {
            UpAxis::Z
        } else {
            UpAxis::Y
        }
    }

    fn set_up_axis(&mut self, up_axis: UpAxis) {
        let z_up = matches!(up_axis, UpAxis::Z) as i32;
        unsafe { isaacsim_usd_set_up_axis(self.handle, z_up) };
    }

    fn define_prim(&mut self, path: &str, type_name: &str) -> Result<(), String> {
        let c_path = cstring(path)?;
        let c_type = cstring(type_name)?;
        if unsafe { isaacsim_usd_define_prim(self.handle, c_path.as_ptr(), c_type.as_ptr()) } {
            Ok(())
        } else {
            Err(format!("{path:?} is not a valid SdfPath"))
        }
    }

    fn prim_exists(&self, path: &str) -> bool {
        let Ok(c_path) = cstring(path) else {
            return false;
        };
        unsafe { isaacsim_usd_prim_exists(self.handle, c_path.as_ptr()) }
    }

    fn type_name(&self, path: &str) -> Option<String> {
        let c_path = cstring(path).ok()?;
        take_string(unsafe { isaacsim_usd_prim_type_name(self.handle, c_path.as_ptr()) })
    }

    fn attribute(&self, path: &str, name: &str) -> Option<Value> {
        let c_path = cstring(path).ok()?;
        let c_name = cstring(name).ok()?;
        let mut out = [0.0f64; 4];
        let mut out_str: *mut c_char = std::ptr::null_mut();
        let tag = unsafe {
            isaacsim_usd_get_attr(
                self.handle,
                c_path.as_ptr(),
                c_name.as_ptr(),
                out.as_mut_ptr(),
                &mut out_str,
            )
        };
        match tag {
            TAG_BOOL => Some(Value::Bool(out[0] != 0.0)),
            TAG_INT => Some(Value::Int(out[0] as i64)),
            TAG_FLOAT => Some(Value::Float(out[0] as f32)),
            TAG_DOUBLE => Some(Value::Double(out[0])),
            TAG_VEC3F => Some(Value::Vec3f([out[0] as f32, out[1] as f32, out[2] as f32])),
            TAG_VEC3D => Some(Value::Vec3d([out[0], out[1], out[2]])),
            TAG_QUATF => Some(Value::Quatf([
                out[0] as f32,
                out[1] as f32,
                out[2] as f32,
                out[3] as f32,
            ])),
            TAG_QUATD => Some(Value::Quatd(out)),
            TAG_TOKEN => take_string(out_str).map(Value::Token),
            TAG_STRING => take_string(out_str).map(Value::String),
            TAG_TOKEN_ARRAY => take_string(out_str).map(|joined| {
                Value::TokenArray(if joined.is_empty() {
                    Vec::new()
                } else {
                    joined.split('\x1f').map(str::to_string).collect()
                })
            }),
            _ => None,
        }
    }

    fn set_attribute(&mut self, path: &str, name: &str, value: Value) -> Result<(), String> {
        let c_path = cstring(path)?;
        let c_name = cstring(name)?;
        let (p, n) = (c_path.as_ptr(), c_name.as_ptr());
        let ok = match &value {
            Value::Bool(v) => unsafe { isaacsim_usd_set_attr_bool(self.handle, p, n, *v) },
            Value::Int(v) => unsafe { isaacsim_usd_set_attr_int(self.handle, p, n, *v) },
            Value::Float(v) => unsafe { isaacsim_usd_set_attr_float(self.handle, p, n, *v) },
            Value::Double(v) => unsafe { isaacsim_usd_set_attr_double(self.handle, p, n, *v) },
            Value::Vec3f(v) => unsafe {
                isaacsim_usd_set_attr_vec3f(self.handle, p, n, v.as_ptr())
            },
            Value::Vec3d(v) => unsafe {
                isaacsim_usd_set_attr_vec3d(self.handle, p, n, v.as_ptr())
            },
            Value::Quatf(q) => unsafe {
                isaacsim_usd_set_attr_quatf(self.handle, p, n, q.as_ptr())
            },
            Value::Quatd(q) => unsafe {
                isaacsim_usd_set_attr_quatd(self.handle, p, n, q.as_ptr())
            },
            Value::Token(v) => {
                let c_value = cstring(v)?;
                unsafe { isaacsim_usd_set_attr_token(self.handle, p, n, c_value.as_ptr()) }
            }
            Value::String(v) => {
                let c_value = cstring(v)?;
                unsafe { isaacsim_usd_set_attr_string(self.handle, p, n, c_value.as_ptr()) }
            }
            Value::TokenArray(tokens) => {
                let c_tokens: Result<Vec<CString>, String> =
                    tokens.iter().map(|t| cstring(t)).collect();
                let c_tokens = c_tokens?;
                let ptrs: Vec<*const c_char> = c_tokens.iter().map(|t| t.as_ptr()).collect();
                let uniform = UNIFORM_ATTRIBUTES.contains(&name);
                unsafe {
                    isaacsim_usd_set_attr_token_array(
                        self.handle,
                        p,
                        n,
                        ptrs.as_ptr(),
                        ptrs.len() as i32,
                        uniform,
                    )
                }
            }
        };
        if ok {
            Ok(())
        } else {
            Err(format!("failed to set attribute {name} on {path}"))
        }
    }

    fn remove_attribute(&mut self, path: &str, name: &str) -> Result<(), String> {
        let c_path = cstring(path)?;
        let c_name = cstring(name)?;
        unsafe { isaacsim_usd_remove_attribute(self.handle, c_path.as_ptr(), c_name.as_ptr()) };
        Ok(())
    }

    fn add_inherit(&mut self, path: &str, source: &str) -> Result<(), String> {
        let c_path = cstring(path)?;
        let c_source = cstring(source)?;
        if unsafe { isaacsim_usd_add_inherit(self.handle, c_path.as_ptr(), c_source.as_ptr()) } {
            Ok(())
        } else {
            Err(format!("failed to add inherit {source} on {path}"))
        }
    }

    fn copy_spec(&mut self, source: &str, dest: &str) -> Result<(), String> {
        let c_source = cstring(source)?;
        let c_dest = cstring(dest)?;
        if unsafe { isaacsim_usd_copy_spec(self.handle, c_source.as_ptr(), c_dest.as_ptr()) } {
            Ok(())
        } else {
            Err(format!("failed to copy spec {source} -> {dest}"))
        }
    }
}
