// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// C ABI shim over the OpenUSD C++ API, exposing the subset of stage
// operations behind isaacsim_scene::SceneStage. Value type tags must match
// src/lib.rs.

#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>

#include <cstdlib>
#include <cstring>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

struct Handle {
    UsdStageRefPtr stage;
};

UsdStageRefPtr stage(void* handle) { return static_cast<Handle*>(handle)->stage; }

char* dup_string(const std::string& s) {
    char* out = static_cast<char*>(std::malloc(s.size() + 1));
    std::memcpy(out, s.c_str(), s.size() + 1);
    return out;
}

// Value type tags shared with the Rust side.
enum ValueTag {
    kNone = 0,
    kBool = 1,
    kInt = 2,
    kFloat = 3,
    kDouble = 4,
    kVec3f = 5,
    kVec3d = 6,
    kQuatf = 7,
    kQuatd = 8,
    kToken = 9,
    kString = 10,
    kTokenArray = 11,
};

}  // namespace

extern "C" {

void* isaacsim_usd_stage_create_in_memory() {
    UsdStageRefPtr s = UsdStage::CreateInMemory();
    return s ? new Handle{s} : nullptr;
}

void* isaacsim_usd_stage_open(const char* path) {
    UsdStageRefPtr s = UsdStage::Open(path);
    return s ? new Handle{s} : nullptr;
}

void isaacsim_usd_stage_free(void* handle) { delete static_cast<Handle*>(handle); }

bool isaacsim_usd_stage_export(void* handle, const char* path) {
    return stage(handle)->Export(path);
}

char* isaacsim_usd_stage_export_string(void* handle) {
    std::string out;
    if (!stage(handle)->ExportToString(&out)) {
        return nullptr;
    }
    return dup_string(out);
}

void isaacsim_usd_string_free(char* s) { std::free(s); }

int isaacsim_usd_up_axis(void* handle) {
    return UsdGeomGetStageUpAxis(stage(handle)) == UsdGeomTokens->z ? 1 : 0;
}

bool isaacsim_usd_set_up_axis(void* handle, int z_up) {
    return UsdGeomSetStageUpAxis(stage(handle), z_up ? UsdGeomTokens->z : UsdGeomTokens->y);
}

bool isaacsim_usd_define_prim(void* handle, const char* path, const char* type_name) {
    if (!SdfPath::IsValidPathString(path)) {
        return false;
    }
    UsdPrim prim = stage(handle)->DefinePrim(SdfPath(path), TfToken(type_name));
    return static_cast<bool>(prim);
}

bool isaacsim_usd_prim_exists(void* handle, const char* path) {
    if (!SdfPath::IsValidPathString(path)) {
        return false;
    }
    return static_cast<bool>(stage(handle)->GetPrimAtPath(SdfPath(path)));
}

// Returns the composed type name, or nullptr if the prim does not exist.
char* isaacsim_usd_prim_type_name(void* handle, const char* path) {
    if (!SdfPath::IsValidPathString(path)) {
        return nullptr;
    }
    UsdPrim prim = stage(handle)->GetPrimAtPath(SdfPath(path));
    if (!prim) {
        return nullptr;
    }
    return dup_string(prim.GetTypeName().GetString());
}

bool isaacsim_usd_add_inherit(void* handle, const char* path, const char* source) {
    UsdPrim prim = stage(handle)->GetPrimAtPath(SdfPath(path));
    if (!prim) {
        return false;
    }
    return prim.GetInherits().AddInherit(SdfPath(source), UsdListPositionFrontOfPrependList);
}

bool isaacsim_usd_copy_spec(void* handle, const char* source, const char* dest) {
    SdfLayerHandle layer = stage(handle)->GetRootLayer();
    SdfPath source_path(source);
    SdfPath dest_path(dest);
    if (!layer->GetPrimAtPath(source_path)) {
        return false;
    }
    SdfCreatePrimInLayer(layer, dest_path);
    return SdfCopySpec(layer, source_path, layer, dest_path);
}

bool isaacsim_usd_remove_attribute(void* handle, const char* path, const char* name) {
    UsdPrim prim = stage(handle)->GetPrimAtPath(SdfPath(path));
    if (!prim) {
        return true;  // removing from a non-existent prim is not an error
    }
    prim.RemoveProperty(TfToken(name));
    return true;
}

// --- attribute setters ---

static UsdAttribute create_attr(void* handle, const char* path, const char* name,
                                const SdfValueTypeName& type, bool uniform) {
    UsdPrim prim = stage(handle)->GetPrimAtPath(SdfPath(path));
    if (!prim) {
        return UsdAttribute();
    }
    return prim.CreateAttribute(TfToken(name), type, /*custom=*/false,
                                uniform ? SdfVariabilityUniform : SdfVariabilityVarying);
}

bool isaacsim_usd_set_attr_bool(void* h, const char* p, const char* n, bool v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Bool, false);
    return a && a.Set(v);
}

bool isaacsim_usd_set_attr_int(void* h, const char* p, const char* n, long long v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Int, false);
    return a && a.Set(static_cast<int>(v));
}

bool isaacsim_usd_set_attr_float(void* h, const char* p, const char* n, float v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Float, false);
    return a && a.Set(v);
}

bool isaacsim_usd_set_attr_double(void* h, const char* p, const char* n, double v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Double, false);
    return a && a.Set(v);
}

bool isaacsim_usd_set_attr_vec3f(void* h, const char* p, const char* n, const float* v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Float3, false);
    return a && a.Set(GfVec3f(v[0], v[1], v[2]));
}

bool isaacsim_usd_set_attr_vec3d(void* h, const char* p, const char* n, const double* v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Double3, false);
    return a && a.Set(GfVec3d(v[0], v[1], v[2]));
}

// Quaternions are passed as [w, x, y, z].
bool isaacsim_usd_set_attr_quatf(void* h, const char* p, const char* n, const float* q) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Quatf, false);
    return a && a.Set(GfQuatf(q[0], q[1], q[2], q[3]));
}

bool isaacsim_usd_set_attr_quatd(void* h, const char* p, const char* n, const double* q) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Quatd, false);
    return a && a.Set(GfQuatd(q[0], q[1], q[2], q[3]));
}

bool isaacsim_usd_set_attr_token(void* h, const char* p, const char* n, const char* v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->Token, false);
    return a && a.Set(TfToken(v));
}

bool isaacsim_usd_set_attr_string(void* h, const char* p, const char* n, const char* v) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->String, false);
    return a && a.Set(std::string(v));
}

bool isaacsim_usd_set_attr_token_array(void* h, const char* p, const char* n,
                                       const char* const* items, int count, bool uniform) {
    UsdAttribute a = create_attr(h, p, n, SdfValueTypeNames->TokenArray, uniform);
    if (!a) {
        return false;
    }
    VtArray<TfToken> tokens(count);
    for (int i = 0; i < count; ++i) {
        tokens[i] = TfToken(items[i]);
    }
    return a.Set(tokens);
}

// --- attribute getter ---

// Returns a ValueTag. Numeric components are written to `out` (up to 4);
// token/string values (and token arrays joined with '\x1f') are returned via
// `out_str` (caller frees with isaacsim_usd_string_free).
int isaacsim_usd_get_attr(void* handle, const char* path, const char* name, double* out,
                          char** out_str) {
    *out_str = nullptr;
    if (!SdfPath::IsValidPathString(path)) {
        return kNone;
    }
    UsdPrim prim = stage(handle)->GetPrimAtPath(SdfPath(path));
    if (!prim) {
        return kNone;
    }
    UsdAttribute attr = prim.GetAttribute(TfToken(name));
    VtValue value;
    if (!attr || !attr.Get(&value)) {
        return kNone;
    }
    if (value.IsHolding<bool>()) {
        out[0] = value.UncheckedGet<bool>() ? 1.0 : 0.0;
        return kBool;
    }
    if (value.IsHolding<int>()) {
        out[0] = value.UncheckedGet<int>();
        return kInt;
    }
    if (value.IsHolding<float>()) {
        out[0] = value.UncheckedGet<float>();
        return kFloat;
    }
    if (value.IsHolding<double>()) {
        out[0] = value.UncheckedGet<double>();
        return kDouble;
    }
    if (value.IsHolding<GfVec3f>()) {
        GfVec3f v = value.UncheckedGet<GfVec3f>();
        out[0] = v[0];
        out[1] = v[1];
        out[2] = v[2];
        return kVec3f;
    }
    if (value.IsHolding<GfVec3d>()) {
        GfVec3d v = value.UncheckedGet<GfVec3d>();
        out[0] = v[0];
        out[1] = v[1];
        out[2] = v[2];
        return kVec3d;
    }
    if (value.IsHolding<GfQuatf>()) {
        GfQuatf q = value.UncheckedGet<GfQuatf>();
        out[0] = q.GetReal();
        out[1] = q.GetImaginary()[0];
        out[2] = q.GetImaginary()[1];
        out[3] = q.GetImaginary()[2];
        return kQuatf;
    }
    if (value.IsHolding<GfQuatd>()) {
        GfQuatd q = value.UncheckedGet<GfQuatd>();
        out[0] = q.GetReal();
        out[1] = q.GetImaginary()[0];
        out[2] = q.GetImaginary()[1];
        out[3] = q.GetImaginary()[2];
        return kQuatd;
    }
    if (value.IsHolding<TfToken>()) {
        *out_str = dup_string(value.UncheckedGet<TfToken>().GetString());
        return kToken;
    }
    if (value.IsHolding<std::string>()) {
        *out_str = dup_string(value.UncheckedGet<std::string>());
        return kString;
    }
    if (value.IsHolding<VtArray<TfToken>>()) {
        const VtArray<TfToken>& tokens = value.UncheckedGet<VtArray<TfToken>>();
        std::string joined;
        for (size_t i = 0; i < tokens.size(); ++i) {
            if (i > 0) {
                joined.push_back('\x1f');
            }
            joined += tokens[i].GetString();
        }
        *out_str = dup_string(joined);
        return kTokenArray;
    }
    return kNone;
}

}  // extern "C"
