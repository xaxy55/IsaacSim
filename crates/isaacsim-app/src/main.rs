// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Isaac Sim application entry point (Rust rewrite).
//!
//! Currently a stub that reports version information; the simulation loop,
//! scene management, and extension system are tracked in ROADMAP.md.

use isaacsim_core_version::get_version;

fn main() {
    let cwd = std::env::current_dir().expect("cannot determine working directory");
    match get_version(&cwd) {
        Some(v) => {
            println!("Isaac Sim {} (Rust rewrite, pre-alpha)", v.core);
            if !v.prerelease.is_empty() {
                println!("  prerelease: {}", v.prerelease);
            }
            if !v.buildtag.is_empty() {
                println!("  build: {}", v.buildtag);
            }
        }
        None => println!("Isaac Sim (Rust rewrite, pre-alpha) — no VERSION file found"),
    }
    println!("See ROADMAP.md for rewrite status.");
}
