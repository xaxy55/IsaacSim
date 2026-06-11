// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Compiles the C++ shim and links OpenUSD when the `openusd` feature is
//! enabled. The OpenUSD install prefix comes from `USD_ROOT`
//! (default `/opt/openusd`).

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=src/shim.cpp");
    println!("cargo:rerun-if-env-changed=USD_ROOT");
    if env::var_os("CARGO_FEATURE_OPENUSD").is_none() {
        return;
    }

    let usd_root = env::var("USD_ROOT").unwrap_or_else(|_| "/opt/openusd".to_string());
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR"));
    let obj = out_dir.join("shim.o");
    let lib = out_dir.join("libisaacsim_usd_shim.a");

    let status = Command::new("c++")
        .args([
            "-std=c++17",
            "-O2",
            "-fPIC",
            "-c",
            "src/shim.cpp",
            "-o",
            obj.to_str().expect("utf8 path"),
            &format!("-I{usd_root}/include"),
        ])
        .status()
        .expect("failed to run c++; a C++17 compiler is required for the openusd feature");
    assert!(status.success(), "failed to compile src/shim.cpp against {usd_root}/include");

    let status = Command::new("ar")
        .args(["rcs", lib.to_str().expect("utf8 path"), obj.to_str().expect("utf8 path")])
        .status()
        .expect("failed to run ar");
    assert!(status.success(), "failed to archive the shim object");

    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=static=isaacsim_usd_shim");
    println!("cargo:rustc-link-search=native={usd_root}/lib");
    for lib in [
        "usd_usd",
        "usd_usdGeom",
        "usd_sdf",
        "usd_tf",
        "usd_gf",
        "usd_vt",
        "usd_plug",
        "usd_arch",
    ] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=tbb");
    // Let test binaries find the USD shared libraries and their plugInfo
    // resources without LD_LIBRARY_PATH.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{usd_root}/lib");
}
