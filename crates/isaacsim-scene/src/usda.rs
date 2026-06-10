// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! `.usda` (USD ASCII) serialization for the in-memory stage.
//!
//! This is the first interop step of the USD backend strategy (see
//! ROADMAP.md): stages authored through [`Stage`] can be written as `.usda`
//! files that real USD/Isaac Sim opens directly, and the subset reader
//! round-trips those files (prims, typed attributes, inherits, up axis).
//!
//! The reader is a *subset* parser: it handles the constructs this crate
//! authors plus common simple assets (def/over/class blocks, single-line
//! attribute values, inherit metadata) and skips attribute metadata,
//! relationships, and other composition arcs it does not model.

use crate::stage::{Stage, UpAxis};
use crate::value::Value;

/// Attribute names that USD declares with `uniform` variability; emitted as
/// `uniform token[]` so real USD does not warn on variability mismatch.
const UNIFORM_ATTRIBUTES: &[&str] = &["xformOpOrder"];

impl Stage {
    /// Serialize this stage as `.usda` text (see [`write_usda`]).
    pub fn to_usda(&self) -> String {
        write_usda(self)
    }

    /// Parse `.usda` text into a stage (see [`parse_usda`]).
    pub fn from_usda(text: &str) -> Result<Stage, String> {
        parse_usda(text)
    }

    /// Write this stage to a `.usda` file.
    pub fn save_usda(&self, path: impl AsRef<std::path::Path>) -> Result<(), String> {
        std::fs::write(path.as_ref(), self.to_usda())
            .map_err(|e| format!("failed to write {}: {e}", path.as_ref().display()))
    }

    /// Load a stage from a `.usda` file.
    pub fn load_usda(path: impl AsRef<std::path::Path>) -> Result<Stage, String> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| format!("failed to read {}: {e}", path.as_ref().display()))?;
        parse_usda(&text)
    }
}

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------

/// Serialize `stage` as `.usda` text.
pub fn write_usda(stage: &Stage) -> String {
    let mut out = String::from("#usda 1.0\n(\n");
    let up_axis = match stage.up_axis() {
        UpAxis::Y => "Y",
        UpAxis::Z => "Z",
    };
    out.push_str(&format!("    upAxis = \"{up_axis}\"\n)\n"));
    for child in stage.children("/") {
        out.push('\n');
        write_prim(stage, child, 0, &mut out);
    }
    out
}

fn write_prim(stage: &Stage, path: &str, indent: usize, out: &mut String) {
    let prim = stage.get_prim(path).expect("authored prim");
    let name = path.rsplit('/').next().expect("non-root path");
    let pad = "    ".repeat(indent);

    out.push_str(&pad);
    if prim.type_name.is_empty() {
        out.push_str(&format!("def \"{name}\""));
    } else {
        out.push_str(&format!("def {} \"{name}\"", prim.type_name));
    }
    if !prim.inherits.is_empty() {
        let targets: Vec<String> = prim.inherits.iter().map(|p| format!("<{p}>")).collect();
        out.push_str(&format!(
            " (\n{pad}    prepend inherits = {}\n{pad})",
            if targets.len() == 1 {
                targets[0].clone()
            } else {
                format!("[{}]", targets.join(", "))
            }
        ));
    }
    out.push('\n');
    out.push_str(&format!("{pad}{{\n"));

    for (attr_name, value) in &prim.attributes {
        out.push_str(&format!(
            "{pad}    {}\n",
            format_attribute(attr_name, value)
        ));
    }

    let children = stage.children(path);
    for (i, child) in children.iter().enumerate() {
        if i > 0 || !prim.attributes.is_empty() {
            out.push('\n');
        }
        write_prim(stage, child, indent + 1, out);
    }

    out.push_str(&format!("{pad}}}\n"));
}

fn format_attribute(name: &str, value: &Value) -> String {
    let uniform = if UNIFORM_ATTRIBUTES.contains(&name) {
        "uniform "
    } else {
        ""
    };
    match value {
        Value::Bool(v) => format!("bool {name} = {v}"),
        Value::Int(v) => format!("int {name} = {v}"),
        Value::Float(v) => format!("float {name} = {}", format_f64(*v as f64)),
        Value::Double(v) => format!("double {name} = {}", format_f64(*v)),
        Value::String(v) => format!("string {name} = \"{}\"", escape(v)),
        Value::Token(v) => format!("{uniform}token {name} = \"{}\"", escape(v)),
        Value::Vec3f(v) => format!(
            "float3 {name} = ({}, {}, {})",
            format_f64(v[0] as f64),
            format_f64(v[1] as f64),
            format_f64(v[2] as f64)
        ),
        Value::Vec3d(v) => format!(
            "double3 {name} = ({}, {}, {})",
            format_f64(v[0]),
            format_f64(v[1]),
            format_f64(v[2])
        ),
        // Quaternion text order is (w, x, y, z), matching Gf
        Value::Quatf(q) => format!(
            "quatf {name} = ({}, {}, {}, {})",
            format_f64(q[0] as f64),
            format_f64(q[1] as f64),
            format_f64(q[2] as f64),
            format_f64(q[3] as f64)
        ),
        Value::Quatd(q) => format!(
            "quatd {name} = ({}, {}, {}, {})",
            format_f64(q[0]),
            format_f64(q[1]),
            format_f64(q[2]),
            format_f64(q[3])
        ),
        Value::TokenArray(tokens) => {
            let items: Vec<String> = tokens.iter().map(|t| format!("\"{}\"", escape(t))).collect();
            format!("{uniform}token[] {name} = [{}]", items.join(", "))
        }
    }
}

fn format_f64(v: f64) -> String {
    // Rust's shortest round-trip Display; USD parses both "0" and "0.0"
    format!("{v}")
}

fn escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

// ---------------------------------------------------------------------------
// Reader (subset)
// ---------------------------------------------------------------------------

/// Parse `.usda` text into a [`Stage`].
///
/// Subset reader: understands `def`/`over`/`class` prim blocks, single-line
/// attribute values of the types in [`Value`], `inherits` prim metadata, and
/// the `upAxis` layer metadata. Unrecognized statements (relationships,
/// references, time samples, attribute metadata, …) are skipped.
pub fn parse_usda(text: &str) -> Result<Stage, String> {
    let mut lines = text.lines().enumerate().peekable();

    let (_, header) = lines.next().ok_or("empty input")?;
    if !header.trim_start().starts_with("#usda") {
        return Err("missing '#usda' header".to_string());
    }

    let mut stage = Stage::new();
    let mut path_stack: Vec<String> = Vec::new();

    while let Some((line_no, raw)) = lines.next() {
        let line = strip_comment(raw).trim().to_string();
        if line.is_empty() {
            continue;
        }
        let err = |msg: &str| format!("line {}: {}", line_no + 1, msg);

        // Layer metadata block right after the header
        if line == "(" && path_stack.is_empty() {
            let mut depth = 1;
            for (_, raw) in lines.by_ref() {
                let meta = strip_comment(raw).trim().to_string();
                depth += paren_delta(&meta);
                if depth == 0 {
                    break;
                }
                if let Some(value) = meta.strip_prefix("upAxis") {
                    match value.trim().trim_start_matches('=').trim() {
                        "\"Y\"" => stage.set_up_axis(UpAxis::Y),
                        "\"Z\"" => stage.set_up_axis(UpAxis::Z),
                        other => return Err(err(&format!("unsupported upAxis {other}"))),
                    }
                }
            }
            continue;
        }

        // Prim declaration: def [Type] "Name" [(metadata)] [{]
        if let Some(decl) = parse_prim_declaration(&line) {
            let (type_name, name, rest) = decl;
            let parent = path_stack.last().map(String::as_str).unwrap_or("");
            let path = format!("{parent}/{name}");
            stage.define_prim(&path, &type_name).map_err(|e| err(&e))?;

            let mut rest = rest.trim().to_string();
            // Prim metadata block (may span lines)
            if rest.starts_with('(') {
                let mut meta = rest.clone();
                let mut depth = paren_delta(&meta);
                while depth > 0 {
                    let (_, raw) = lines.next().ok_or_else(|| err("unterminated prim metadata"))?;
                    let cont = strip_comment(raw).trim().to_string();
                    depth += paren_delta(&cont);
                    meta.push('\n');
                    meta.push_str(&cont);
                }
                // The body brace may share the metadata's closing line
                rest = meta
                    .rsplit(')')
                    .next()
                    .unwrap_or("")
                    .trim()
                    .to_string();
                for target in parse_inherits(&meta) {
                    let prim = stage.get_prim_mut(&path).expect("just defined");
                    if !prim.inherits.contains(&target) {
                        prim.inherits.push(target);
                    }
                }
            }
            // Opening brace on this line or the next
            if !rest.starts_with('{') {
                let (_, raw) = lines.next().ok_or_else(|| err("expected '{' after prim declaration"))?;
                if !strip_comment(raw).trim().starts_with('{') {
                    return Err(err("expected '{' after prim declaration"));
                }
            }
            path_stack.push(path);
            continue;
        }

        if line == "}" {
            if path_stack.pop().is_none() {
                return Err(err("unbalanced '}'"));
            }
            continue;
        }

        // Attribute line inside a prim body
        if let Some(prim_path) = path_stack.last() {
            if let Some((name, value)) = parse_attribute(&line) {
                stage
                    .set_attribute(prim_path, &name, value)
                    .map_err(|e| err(&e))?;
            }
            // Unrecognized statements (rel, references, declarations without
            // values, …) are skipped by the subset reader.
            continue;
        }

        return Err(err(&format!("unexpected statement outside a prim: {line:?}")));
    }

    if !path_stack.is_empty() {
        return Err(format!("unterminated prim block for {}", path_stack.last().unwrap()));
    }
    Ok(stage)
}

fn strip_comment(line: &str) -> &str {
    // '#' starts a comment outside of quoted strings
    let mut in_string = false;
    let mut prev_escape = false;
    for (i, c) in line.char_indices() {
        match c {
            '"' if !prev_escape => in_string = !in_string,
            '#' if !in_string => return &line[..i],
            _ => {}
        }
        prev_escape = c == '\\' && !prev_escape;
    }
    line
}

fn paren_delta(line: &str) -> i32 {
    let mut in_string = false;
    let mut prev_escape = false;
    let mut delta = 0;
    for c in line.chars() {
        match c {
            '"' if !prev_escape => in_string = !in_string,
            '(' if !in_string => delta += 1,
            ')' if !in_string => delta -= 1,
            _ => {}
        }
        prev_escape = c == '\\' && !prev_escape;
    }
    delta
}

/// Parse `def [Type] "Name" rest`, also accepting `over` and `class`.
fn parse_prim_declaration(line: &str) -> Option<(String, String, String)> {
    let rest = ["def ", "over ", "class "]
        .iter()
        .find_map(|kw| line.strip_prefix(kw))
        .or_else(|| ["def\"", "over\"", "class\""].iter().find_map(|kw| {
            line.strip_prefix(&kw[..kw.len() - 1])
        }))?;
    let rest = rest.trim_start();
    let (type_name, rest) = if let Some(stripped) = rest.strip_prefix('"') {
        (String::new(), format!("\"{stripped}"))
    } else {
        let (t, r) = rest.split_once(char::is_whitespace)?;
        (t.to_string(), r.trim_start().to_string())
    };
    let rest = rest.strip_prefix('"')?;
    let (name, rest) = rest.split_once('"')?;
    Some((type_name, name.to_string(), rest.to_string()))
}

/// Extract inherit targets (`</path>`) from a prim metadata block that
/// contains an `inherits =` statement (with optional `prepend`/`append`).
fn parse_inherits(metadata: &str) -> Vec<String> {
    let Some(idx) = metadata.find("inherits") else {
        return Vec::new();
    };
    let after = &metadata[idx..];
    let Some(eq) = after.find('=') else {
        return Vec::new();
    };
    // Targets end at the end of the statement (newline or closing paren)
    let value = after[eq + 1..]
        .split(['\n', ')'])
        .next()
        .unwrap_or("");
    value
        .split(['[', ']', ','])
        .filter_map(|item| {
            let item = item.trim();
            item.strip_prefix('<')?.strip_suffix('>').map(str::to_string)
        })
        .collect()
}

/// Parse `[uniform] [custom] <type> name = value` for the supported types.
/// Returns `None` for declarations without values or unsupported constructs.
fn parse_attribute(line: &str) -> Option<(String, Value)> {
    let mut rest = line;
    for modifier in ["uniform ", "custom ", "varying "] {
        if let Some(stripped) = rest.strip_prefix(modifier) {
            rest = stripped;
        }
    }
    let (type_name, rest) = rest.split_once(char::is_whitespace)?;
    let (name, value_text) = rest.split_once('=')?;
    let name = name.trim();
    // Strip trailing attribute metadata: `= value (doc = "...")`
    let value_text = value_text.trim();
    let value = match type_name {
        "bool" => Value::Bool(value_text.parse().ok()?),
        "int" => Value::Int(value_text.parse().ok()?),
        "float" => Value::Float(value_text.parse().ok()?),
        "double" => Value::Double(value_text.parse().ok()?),
        "string" => Value::String(parse_quoted(value_text)?),
        "token" => Value::Token(parse_quoted(value_text)?),
        "float3" | "point3f" | "normal3f" | "color3f" => {
            let v = parse_tuple(value_text, 3)?;
            Value::Vec3f([v[0] as f32, v[1] as f32, v[2] as f32])
        }
        "double3" | "point3d" => {
            let v = parse_tuple(value_text, 3)?;
            Value::Vec3d([v[0], v[1], v[2]])
        }
        "quatf" => {
            let v = parse_tuple(value_text, 4)?;
            Value::Quatf([v[0] as f32, v[1] as f32, v[2] as f32, v[3] as f32])
        }
        "quatd" => {
            let v = parse_tuple(value_text, 4)?;
            Value::Quatd([v[0], v[1], v[2], v[3]])
        }
        "token[]" => {
            let inner = value_text.strip_prefix('[')?.rsplit_once(']')?.0;
            let tokens: Option<Vec<String>> = inner
                .split(',')
                .filter(|s| !s.trim().is_empty())
                .map(|s| parse_quoted(s.trim()))
                .collect();
            Value::TokenArray(tokens?)
        }
        _ => return None,
    };
    Some((name.to_string(), value))
}

fn parse_quoted(text: &str) -> Option<String> {
    let inner = text.trim().strip_prefix('"')?.rsplit_once('"')?.0;
    Some(inner.replace("\\\"", "\"").replace("\\\\", "\\"))
}

fn parse_tuple(text: &str, n: usize) -> Option<Vec<f64>> {
    let inner = text.trim().strip_prefix('(')?.rsplit_once(')')?.0;
    let values: Option<Vec<f64>> = inner.split(',').map(|s| s.trim().parse().ok()).collect();
    let values = values?;
    if values.len() == n {
        Some(values)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_stage() -> Stage {
        let mut stage = Stage::new();
        stage.define_prim("/World", "Xform").unwrap();
        stage.define_prim("/World/Cube_0", "Cube").unwrap();
        stage
            .set_attribute("/World/Cube_0", "xformOp:translate", Value::Vec3d([1.0, 2.0, 3.0]))
            .unwrap();
        stage
            .set_attribute("/World/Cube_0", "xformOp:orient", Value::Quatd([1.0, 0.0, 0.0, 0.0]))
            .unwrap();
        stage
            .set_attribute(
                "/World/Cube_0",
                "xformOpOrder",
                Value::TokenArray(vec!["xformOp:translate".into(), "xformOp:orient".into()]),
            )
            .unwrap();
        stage.define_prim("/World/Cube_1", "").unwrap();
        stage
            .get_prim_mut("/World/Cube_1")
            .unwrap()
            .inherits
            .push("/World/Cube_0".to_string());
        stage
    }

    #[test]
    fn test_write_usda_golden() {
        let expected = r#"#usda 1.0
(
    upAxis = "Z"
)

def Xform "World"
{
    def Cube "Cube_0"
    {
        quatd xformOp:orient = (1, 0, 0, 0)
        double3 xformOp:translate = (1, 2, 3)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
    }

    def "Cube_1" (
        prepend inherits = </World/Cube_0>
    )
    {
    }
}
"#;
        assert_eq!(write_usda(&sample_stage()), expected);
    }

    #[test]
    fn test_round_trip() {
        let mut stage = sample_stage();
        stage.set_up_axis(UpAxis::Y);
        stage
            .set_attribute("/World", "flag", Value::Bool(true))
            .unwrap();
        stage.set_attribute("/World", "count", Value::Int(-3)).unwrap();
        stage
            .set_attribute("/World", "ratio", Value::Float(0.5))
            .unwrap();
        stage
            .set_attribute("/World", "mass", Value::Double(2.25))
            .unwrap();
        stage
            .set_attribute("/World", "label", Value::String("a \"b\" \\c".into()))
            .unwrap();
        stage
            .set_attribute("/World", "kind", Value::Token("group".into()))
            .unwrap();
        stage
            .set_attribute("/World", "velocity", Value::Vec3f([0.5, -1.0, 2.0]))
            .unwrap();
        stage
            .set_attribute("/World", "rot", Value::Quatf([0.0, 0.0, 0.0, 1.0]))
            .unwrap();

        let parsed = parse_usda(&write_usda(&stage)).unwrap();
        assert_eq!(parsed.up_axis(), stage.up_axis());

        let original: Vec<_> = stage.prims().collect();
        let round_tripped: Vec<_> = parsed.prims().collect();
        assert_eq!(original.len(), round_tripped.len());
        for ((path_a, prim_a), (path_b, prim_b)) in original.iter().zip(&round_tripped) {
            assert_eq!(path_a, path_b);
            assert_eq!(prim_a.type_name, prim_b.type_name, "type mismatch at {path_a}");
            assert_eq!(prim_a.inherits, prim_b.inherits, "inherits mismatch at {path_a}");
            assert_eq!(prim_a.attributes, prim_b.attributes, "attrs mismatch at {path_a}");
        }
    }

    #[test]
    fn test_parse_external_subset() {
        // Constructs as authored by Omniverse / pxr tooling: layer metadata
        // we ignore, comments, over specs, multiple inherits, attribute
        // declarations without values, relationships, unknown types.
        let text = r#"#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
)

# a comment
def Xform "World" (
    kind = "group"
)
{
    double3 xformOp:translate = (0, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    rel physics:body = </World/Cube>
    float3 customColor = (0.1, 0.2, 0.3)
    double radius

    over "Child" (
        prepend inherits = [</World/A>, </World/B>]
    )
    {
        quatf xformOp:orient = (0.707, 0.707, 0, 0)  # trailing comment
        matrix4d unknownType = ( (1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1) )
    }
}
"#;
        let stage = parse_usda(text).unwrap();
        assert_eq!(stage.up_axis(), UpAxis::Y);
        assert_eq!(stage.composed_type_name("/World").unwrap(), "Xform");
        assert_eq!(
            stage.get_attribute("/World", "xformOp:translate"),
            Some(&Value::Vec3d([0.0, 0.0, 0.0]))
        );
        assert_eq!(
            stage.get_attribute("/World", "customColor"),
            Some(&Value::Vec3f([0.1, 0.2, 0.3]))
        );
        // Valueless declarations and unknown constructs are skipped
        assert_eq!(stage.get_attribute("/World", "radius"), None);
        let child = stage.get_prim("/World/Child").unwrap();
        assert_eq!(child.inherits, vec!["/World/A".to_string(), "/World/B".to_string()]);
        assert!(matches!(
            stage.get_attribute("/World/Child", "xformOp:orient"),
            Some(Value::Quatf(_))
        ));
        assert_eq!(stage.get_attribute("/World/Child", "unknownType"), None);
    }

    /// Verbatim output of pxr USD 26.5 (`Usd.Stage.CreateNew` + UsdGeom
    /// authoring); the reader must accept real USD-authored files.
    #[test]
    fn test_parse_pxr_authored_file() {
        let text = r#"#usda 1.0
(
    upAxis = "Y"
)

def Xform "Robot"
{
    quatf xformOp:orient = (0.707, 0.707, 0, 0)
    double3 xformOp:translate = (1, 2, 3)
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

    def Cube "Body"
    {
        double size = 2
    }
}
"#;
        let stage = parse_usda(text).unwrap();
        assert_eq!(stage.up_axis(), UpAxis::Y);
        assert_eq!(stage.composed_type_name("/Robot").unwrap(), "Xform");
        assert_eq!(
            stage.get_attribute("/Robot", "xformOp:translate"),
            Some(&Value::Vec3d([1.0, 2.0, 3.0]))
        );
        assert_eq!(
            stage.get_attribute("/Robot", "xformOp:orient"),
            Some(&Value::Quatf([0.707, 0.707, 0.0, 0.0]))
        );
        assert_eq!(
            stage.get_attribute("/Robot/Body", "size"),
            Some(&Value::Double(2.0))
        );
    }

    #[test]
    fn test_parse_errors() {
        assert!(parse_usda("").is_err());
        assert!(parse_usda("not a usda file").is_err());
        assert!(parse_usda("#usda 1.0\ndef Xform \"World\"\n{\n").is_err());
        assert!(parse_usda("#usda 1.0\n}\n").is_err());
    }
}
