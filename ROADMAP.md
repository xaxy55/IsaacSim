# Isaac Sim → Rust Rewrite Roadmap

This document tracks the status of rewriting Isaac Sim in Rust. The original
C++/Python/Omniverse-Kit codebase has been moved to [`legacy/`](legacy/) and
remains the reference implementation. New Rust code lives in
[`crates/`](crates/) as a Cargo workspace.

**Last updated:** 2026-06-10

## Status legend

| Symbol | Meaning |
|--------|---------|
| ✅ | Ported — Rust implementation with tests, behavior verified against legacy |
| 🚧 | In progress |
| 📋 | Planned — scoped but not started |
| ⬜ | Not started |
| ⛔ | Blocked — depends on closed-source or external components (see notes) |

## Reality check / constraints

Isaac Sim is not a self-contained codebase. The `legacy/` tree (~2,300 Python
files, ~380 C++ files across 123 Kit extensions) is the *open* layer on top of
closed-source NVIDIA components that cannot be rewritten from this repository:

- **Omniverse Kit** — the application framework, extension loader, UI (omni.ui)
- **RTX renderer** — the rendering stack
- **PhysX / Newton solvers** — physics engines (Newton is open source and could
  be bound, not rewritten)
- **OpenUSD** — scene representation (open source; the Rust strategy is to use
  [`openusd-rs`](https://crates.io/crates/openusd) bindings or FFI to the C++
  library, not a rewrite)

The strategy is therefore: **port the Isaac Sim logic layer to Rust**
(controllers, kinematics, utilities, importers/exporters, data pipelines,
networking bridges), and **bind** to external engines (USD, physics, rendering)
behind Rust traits so backends can be swapped later.

## Phases

### Phase 0 — Foundation (this PR)
- [x] Move legacy codebase to `legacy/`
- [x] Cargo workspace scaffolding (`crates/`)
- [x] CI-friendly build (`cargo build && cargo test` from repo root)
- [x] First ports: `isaacsim.core.version`, wheeled-robot controllers
- [x] `isaacsim` CLI binary stub

### Phase 1 — Core utilities (pure logic, no Kit/USD dependency) ✅
Math/transform utilities, version/config handling, pure-Python algorithm ports.
- [x] `isaacsim-core-version` — version parsing/retrieval
- [x] `isaacsim-core-math` — transform utilities (quaternion/Euler/rotation-matrix
  conversions, look-at, relative transforms) from `isaacsim.core.experimental.utils`
- [x] `isaacsim-robot-wheeled` — all five controllers (differential, Ackermann,
  holonomic, Stanley, quintic planner)
- [x] Remaining Phase 1 rows triaged as n/a (Python import shims, C++ headers,
  Kit test scaffolding — no Rust equivalent needed)

### Phase 2 — Scene & simulation core 🚧
USD bindings, simulation manager, cloner, prim wrappers.
- [x] `isaacsim-scene` — in-memory stage/prim data model (paths, typed
  attributes, inherit composition); the native backend behind which a real
  USD backend can be bound later
- [x] `isaacsim-core-cloner` — Cloner + GridCloner on top of `isaacsim-scene`
- [x] **USD strategy decided:** FFI to the C++ OpenUSD library behind the
  `isaacsim-scene` API (`openusd-rs` is not yet complete enough for
  composition/physics schemas); the in-memory backend remains for tests
- [x] `.usda` interop — `isaacsim-scene::usda` writer + subset reader.
  Rust-authored stages verified against real USD (pxr 26.5 opens exported
  files; inherit composition, types, and transforms resolve identically;
  pxr-authored files parse back)
- [x] `SceneStage` backend trait — the cloner and future logic-layer code
  are generic over the backend; includes relationships, child traversal,
  and TRS world-transform composition (`isaacsim-scene::xform`)
- [x] `isaacsim-usd` — OpenUSD FFI binding crate (C ABI shim + Rust
  `UsdStage` implementing `SceneStage`, behind the `openusd` feature with
  `USD_ROOT` pointing at an OpenUSD install). Verified: the ported cloner
  runs unchanged on real USD stages (built from OpenUSD v25.11 source,
  imaging/python off) and both backends produce identical results
- [ ] Simulation manager, prim wrappers, remaining Phase 2 rows

### Phase 3 — Robotics stack 🚧
Importers (URDF/MJCF), motion generation, robot setup tools, sensors.
- [x] `isaacsim-asset-urdf` — URDF parsing layer: XML → robot model
  (links, joints, limits, dynamics, mimic, safety, materials, geometry),
  `merge_fixed_joints` pre-processing (transform composition + parallel-axis
  inertia merging), and a URDF writer. Legacy `test_urdf_utils.py` ported;
  legacy fixture files parse
- [x] URDF → stage converter (`convert_urdf_to_stage`) — rigid-body subset
  of the external `urdf_usd_converter`: links at zero-config world poses,
  UsdGeom visuals/collisions, UsdPhysics joints with axis alignment and
  degree limits, MassAPI with principal-axes inertia. Output verified with
  real USD (UsdPhysics schema accessors, world-transform composition) both
  via `.usda` export and authored directly through the FFI backend.
  Mesh tessellation and drives stay downstream (⛔)

### Phase 4 — Connectivity & pipelines
ROS 2 bridge, UCX, streaming, replicator/synthetic-data writers.

### Phase 5 — Application layer
App shell, GUI, examples, benchmarking. Blocked on a UI framework decision
(legacy uses omni.ui, which only exists inside Kit).

## Module status

One row per legacy extension (`legacy/source/extensions/<name>`). Rust crate
names map `isaacsim.foo.bar` → `isaacsim-foo-bar`.

### isaacsim.core (14)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.core.version | `isaacsim-core-version` | 1 | ✅ |
| isaacsim.core.cloner | `isaacsim-core-cloner` (+ `isaacsim-scene`) | 2 | ✅ clone/grid-clone with inherit & copy semantics, legacy tests ported; PhysX replication, collision filtering, Fabric paths ⛔ PhysX/Kit |
| isaacsim.core.deprecation_manager | — | 1 | n/a — Python import shim around carb/Kit; no Rust equivalent needed |
| isaacsim.core.experimental.actuators | — | 2 | ⬜ |
| isaacsim.core.experimental.materials | — | 2 | ⬜ |
| isaacsim.core.experimental.objects | — | 2 | ⬜ |
| isaacsim.core.experimental.primdata | — | 2 | ⬜ |
| isaacsim.core.experimental.prims | — | 2 | ⬜ |
| isaacsim.core.experimental.utils | `isaacsim-core-math` (transform subset) | 1 | ✅ transform module ported with legacy tests; remaining modules (stage, prim, xform, ops, …) are USD/Kit-bound → Phase 2 |
| isaacsim.core.includes | — (C++ headers; superseded by crate APIs) | 1 | n/a |
| isaacsim.core.nodes | — | 2 | ⬜ |
| isaacsim.core.rendering_manager | — | 5 | ⛔ RTX |
| isaacsim.core.simulation_manager | — | 2 | ⬜ |
| isaacsim.core.throttling | — | 2 | ⬜ |

### isaacsim.robot / robot_motion / robot_setup (22)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.robot.experimental.wheeled_robots | `isaacsim-robot-wheeled` | 1 | ✅ all five controllers + `HolonomicRobotUsdSetup` stage reader (runs on in-memory and OpenUSD backends); `WheeledRobot` runtime wrapper needs the simulation core |
| isaacsim.robot.experimental.manipulators.examples | — | 3 | ⬜ |
| isaacsim.robot.policy.examples | — | 3 | ⬜ |
| isaacsim.robot.poser (+ .ui) | — | 3 | ⬜ |
| isaacsim.robot.schema (+ .ui) | — | 3 | ⬜ |
| isaacsim.robot.surface_gripper (+ .ui) | — | 3 | ⬜ |
| isaacsim.robot.wheeled_robots.nodes (+ .ui) | — | 3 | ⬜ |
| isaacsim.robot_motion.cumotion (+ .examples) | — | 3 | ⛔ cuMotion (CUDA) |
| isaacsim.robot_motion.experimental.motion_generation | — | 3 | ⬜ |
| isaacsim.robot_motion.pink (+ .examples) | — | 3 | ⬜ |
| isaacsim.robot_motion.schema | — | 3 | ⬜ |
| isaacsim.robot_setup.assembler | — | 3 | ⬜ |
| isaacsim.robot_setup.collision_detector | — | 3 | ⬜ |
| isaacsim.robot_setup.gain_tuner | — | 3 | ⬜ |
| isaacsim.robot_setup.grasp_editor | — | 3 | ⬜ |
| isaacsim.robot_setup.xrdf_editor | — | 3 | ⬜ |

### isaacsim.asset (16)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.asset.exporter.urdf (+ .ui) | — | 3 | ⬜ |
| isaacsim.asset.gen.conveyor (+ .ui) | — | 3 | ⬜ |
| isaacsim.asset.gen.omap (+ .ui) | — | 3 | ⬜ |
| isaacsim.asset.importer.heightmap | — | 3 | ⬜ |
| isaacsim.asset.importer.mjcf (+ .ui) | — | 3 | ⛔ parsing lives in the external `mujoco-usd-converter` package; the in-repo layer is Kit/USD orchestration only — nothing to port |
| isaacsim.asset.importer.urdf (+ .ui) | `isaacsim-asset-urdf` | 3 | ✅ parser, merge_fixed_joints, writer, and rigid-body URDF→stage converter verified against real UsdPhysics; mesh tessellation + drives ⛔ |
| isaacsim.asset.importer.utils | — | 3 | ⬜ |
| isaacsim.asset.transformer (+ .rules, .ui) | — | 3 | ⬜ |
| isaacsim.asset.validation | — | 3 | ⬜ |

### isaacsim.sensors (8)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.sensors.camera.ui | — | 5 | ⬜ |
| isaacsim.sensors.experimental.physics | — | 3 | ⬜ |
| isaacsim.sensors.experimental.rtx | — | 3 | ⛔ RTX |
| isaacsim.sensors.physics.examples / .nodes / .ui | — | 3 | ⬜ |
| isaacsim.sensors.rtx.nodes / .ui | — | 3 | ⛔ RTX |

### isaacsim.ros2 / ucx / streaming (12)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.ros2.bridge / .core / .nodes / .examples | — | 4 | ⬜ (candidate: `r2r` or `rclrs`) |
| isaacsim.ros2.sim_control / .tf_viewer / .ui / .urdf | — | 4 | ⬜ |
| isaacsim.ucx.bridge / .core / .nodes | — | 4 | ⬜ |
| isaacsim.streaming.rtsp | — | 4 | ⬜ |

### isaacsim.replicator (15)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.replicator.* (behavior, episode_recorder, examples, domain_randomization, mobility_gen, grasping, synthetic_recorder, teleop, writers + ui variants) | — | 4 | ⬜ depends on omni.replicator (Kit) |

### isaacsim.physics (3)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.physics.newton (+ .tensors, .ui) | — | 2 | ⬜ bind Newton, don't rewrite |

### App / GUI / examples / misc (33)

| Legacy extension | Rust crate | Phase | Status |
|---|---|---|---|
| isaacsim.app.about / .setup | `isaacsim-app` (CLI stub) | 5 | 🚧 |
| isaacsim.simulation_app | — | 5 | ⬜ |
| isaacsim.gui.* (5) | — | 5 | ⛔ omni.ui — needs Rust UI framework decision |
| isaacsim.examples.* (5) | — | 5 | ⬜ |
| isaacsim.benchmark.services | — | 5 | ⬜ |
| isaacsim.code_editor.* (3) | — | 4 | ⬜ |
| isaacsim.hsb.* (3) | — | 4 | ⬜ |
| isaacsim.storage.native | — | 2 | ⬜ |
| isaacsim.test.* (3) | — (replaced by `cargo test`) | 1 | n/a |
| isaacsim.util.camera_inspector / .physics | — | 3 | ⬜ |
| isaacsim.pip.newton, omni.pip.* | — (replaced by Cargo dependencies) | — | n/a |
| omni.isaac.core_archive, omni.kit.loop-isaac | — | 2 | ⬜ |
| omni.usd.schema.mujoco / .newton | — | 2 | ⬜ USD schemas |

## How to update this file

When you port a module: create the crate under `crates/`, add it to the
workspace `Cargo.toml`, port the legacy tests, then flip the row here to ✅
with the crate name and bump **Last updated**.
