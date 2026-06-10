// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Version parsing and retrieval utilities for Isaac Sim.
//!
//! Rust port of the legacy `isaacsim.core.version` extension
//! (`legacy/source/extensions/isaacsim.core.version`).

use std::path::{Path, PathBuf};

/// Parsed version information.
///
/// Version strings follow semantic versioning patterns like
/// `"1.2.3-alpha.1+build123"`, broken down into their constituent parts.
/// Components not present in the input remain empty strings, matching the
/// legacy Python behavior.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Version {
    pub core: String,
    pub prerelease: String,
    pub major: String,
    pub minor: String,
    pub patch: String,
    pub pretag: String,
    pub prebuild: String,
    pub buildtag: String,
}

/// Error returned when a version string cannot be parsed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseVersionError(pub String);

impl std::fmt::Display for ParseVersionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "invalid version string: {}", self.0)
    }
}

impl std::error::Error for ParseVersionError {}

/// Parse a full version string (as read from a `VERSION` file) into a [`Version`].
///
/// The legacy Python implementation raises `ValueError` on malformed input;
/// this port returns [`ParseVersionError`] instead.
pub fn parse_version(full_version: &str) -> Result<Version, ParseVersionError> {
    let mut parsed = Version::default();
    let mut rest = full_version;

    if let Some((head, buildtag)) = rest.split_once('+') {
        parsed.buildtag = buildtag.to_string();
        rest = head;
    }
    if let Some((core, prerelease)) = rest.split_once('-') {
        parsed.core = core.to_string();
        parsed.prerelease = prerelease.to_string();
        let mut core_parts = parsed.core.splitn(3, '.');
        parsed.major = next_part(&mut core_parts, full_version)?;
        parsed.minor = next_part(&mut core_parts, full_version)?;
        parsed.patch = next_part(&mut core_parts, full_version)?;
        let (pretag, prebuild) = parsed
            .prerelease
            .split_once('.')
            .ok_or_else(|| ParseVersionError(full_version.to_string()))?;
        parsed.pretag = pretag.to_string();
        parsed.prebuild = prebuild.to_string();
    } else {
        let mut core_parts = rest.splitn(3, '.');
        parsed.major = next_part(&mut core_parts, full_version)?;
        parsed.minor = next_part(&mut core_parts, full_version)?;
        parsed.patch = next_part(&mut core_parts, full_version)?;
        parsed.core = rest.to_string();
    }
    Ok(parsed)
}

fn next_part<'a>(
    parts: &mut impl Iterator<Item = &'a str>,
    full: &str,
) -> Result<String, ParseVersionError> {
    parts
        .next()
        .map(str::to_string)
        .ok_or_else(|| ParseVersionError(full.to_string()))
}

/// Retrieve the application version from a `VERSION` file.
///
/// Looks in `$ISAAC_PATH` if set, otherwise in `app_start_folder`. Returns
/// `None` if the file is missing or unparsable (the legacy implementation
/// returns a tuple of empty strings in that case).
pub fn get_version(app_start_folder: impl AsRef<Path>) -> Option<Version> {
    let base: PathBuf = std::env::var_os("ISAAC_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| app_start_folder.as_ref().to_path_buf());
    let version_file = base.join("VERSION");
    let contents = std::fs::read_to_string(version_file).ok()?;
    let first_line = contents.lines().next()?.trim();
    parse_version(first_line).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Port of legacy `tests/test_version.py::test_version`.
    #[test]
    fn test_version() {
        let parsed = parse_version("2000.0.0-beta.0+branch.0.hash.local").unwrap();
        assert_eq!(parsed.core, "2000.0.0");
        assert_eq!(parsed.pretag, "beta");
        assert_eq!(parsed.prebuild, "0");
        assert_eq!(parsed.buildtag, "branch.0.hash.local");
    }

    #[test]
    fn test_plain_release() {
        let parsed = parse_version("4.5.0").unwrap();
        assert_eq!(parsed.core, "4.5.0");
        assert_eq!(parsed.major, "4");
        assert_eq!(parsed.minor, "5");
        assert_eq!(parsed.patch, "0");
        assert_eq!(parsed.prerelease, "");
        assert_eq!(parsed.buildtag, "");
    }

    #[test]
    fn test_repo_version_file_parses() {
        let parsed = parse_version("6.0.0-rc.59").unwrap();
        assert_eq!(parsed.core, "6.0.0");
        assert_eq!(parsed.pretag, "rc");
        assert_eq!(parsed.prebuild, "59");
    }

    #[test]
    fn test_malformed_version() {
        assert!(parse_version("6.0").is_err());
        assert!(parse_version("6.0.0-rc").is_err());
    }

    #[test]
    fn test_get_version_missing_file() {
        assert!(get_version("/nonexistent/path").is_none());
    }
}
