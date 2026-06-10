// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Prim path utilities (the subset of `Sdf.Path` the ported code needs).
//!
//! Paths are absolute prim paths: `/` followed by `/`-separated identifiers,
//! where each identifier starts with a letter or underscore and continues
//! with letters, digits, or underscores (e.g. `/World/envs/env_0`).

/// Check whether `path` is a valid absolute prim path string
/// (`Sdf.Path.IsValidPathString` for the prim-path subset).
pub fn is_valid_path_string(path: &str) -> bool {
    if path == "/" {
        return true;
    }
    let Some(rest) = path.strip_prefix('/') else {
        return false;
    };
    !rest.is_empty()
        && rest.split('/').all(|segment| {
            let mut chars = segment.chars();
            match chars.next() {
                Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                    chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
                }
                _ => false,
            }
        })
}

/// Return the parent path of `path`, or `None` for the pseudo-root `/`.
///
/// The parent of a top-level prim (e.g. `/World`) is `/`.
pub fn parent_path(path: &str) -> Option<&str> {
    if path == "/" {
        return None;
    }
    match path.rfind('/') {
        Some(0) => Some("/"),
        Some(idx) => Some(&path[..idx]),
        None => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_valid_path_string() {
        assert!(is_valid_path_string("/"));
        assert!(is_valid_path_string("/World"));
        assert!(is_valid_path_string("/World/envs/env_0"));
        assert!(is_valid_path_string("/_private/x1"));
        assert!(!is_valid_path_string(""));
        assert!(!is_valid_path_string("World"));
        assert!(!is_valid_path_string("/World/"));
        assert!(!is_valid_path_string("//World"));
        assert!(!is_valid_path_string("/World/1env"));
        assert!(!is_valid_path_string("not a valid path!!!"));
    }

    #[test]
    fn test_parent_path() {
        assert_eq!(parent_path("/World/envs/env_0"), Some("/World/envs"));
        assert_eq!(parent_path("/World"), Some("/"));
        assert_eq!(parent_path("/"), None);
    }
}
