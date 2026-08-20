# Changelog

All notable changes to the isaacsim-mcp-server project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed / Changed — tool hardening for agent use
- step_simulation now fails loud on a running timeline and the debug loop is
  documented as step-only (never play while debugging). (#1)
- create_action_graph gains inline_script= one-step shortcut; the broken inline
  example is removed. (#2)
- reload_script recompiles Action-Graph ScriptNodes that reference the edited
  file, instead of silently no-oping. (#3)
- get_isaac_logs: eager listener, run-scoped (since_last_play default),
  non-destructive default, and captures print() as [PRINT]. (#4/#5)
- execute_script documents that it can silently disturb a live ScriptNode. (#6)
- create_object documents that scale= is a raw native-size multiplier. (#7)
- stop_simulation resets the scene to spawn state. (#8)

### Fixed — silent wrong answers found by live testing on 5.1.0 and 6.0.1

Each of these was reproduced against a running simulator, one version at a
time, and re-measured after the fix. Unit tests passed throughout: every one of
them needs a real stage, a real physics step or a real referenced asset to show
up at all.

- **Joint limits were reported in degrees while positions were radians.**
  `get_joint_config` returned USD's raw revolute limits: FR3 joint 1 read
  `[-157.2, 157.2]` next to `actual_position=0.5`, where the real limit is
  ±2.7437 rad. An agent clamping a target to those limits would command 25
  revolutions. Limits now arrive in the same units as positions, with an
  explicit `limit_units` per joint. Prismatic limits are deliberately untouched
  — USD stores those in stage units, and converting them turns a 0.04 m gripper
  stroke into 0.0007. `get_robot_info` previously advertised `"degrees"` for the
  same attributes and now agrees. Both adapters. (`adapters/units.py`)
- **A requested rotation compounded with the prim's existing orientation.**
  `set_prim_transform` only ever wrote `xformOp:rotateXYZ`, so on a prim
  carrying `xformOp:orient` the rotation was appended rather than replacing
  anything, landing after `xformOp:scale`. On a prim with orient=90° and
  scale=(1,2,1), asking for 45° produced 135° and a shear of 1.5. It already
  bit in practice: 5.1 cameras ship `orient=(0.5,0.5,-0.5,-0.5)`, so
  `create_camera(rotation=...)` could not aim a camera at all. Both adapters.
  (`adapters/transforms.py`)
- **`get_prim_info` had no rotation.** It returned position only, so "is this
  prim rotated?" was unanswerable through the tools while the docstring
  advertised the transform. It now reports rotation (XYZ degrees, the order
  `transform_object` accepts) and scale, read off the orthonormalized matrix so
  scale cannot corrupt the angle.
- **Environments lost their axis and unit conversion.** USD authors
  `unitsResolve` ops for a reference whose layer declares a different `upAxis`
  or `metersPerUnit` — but only when the target prim has no pre-existing
  children. `load_environment` referenced onto `/Environment`, which ships
  `defaultLight`, so the conversion was skipped: a ground standing on edge,
  10 km across, floor at z=-5000. That is 6 of 25 shipped environments on 5.1
  and 8 of 28 on 6.0 by up-axis, 8 and 10 by units. It now references onto
  `/Environment/<name>` and reports what USD applied under `corrections`, plus
  `bounds` with extent and floor height. Both adapters.
- **`clear_scene` did not clear a loaded environment**, so a later
  `create_physics_scene(floor=True)` stacked a second ground under the first.
  It now empties `/Environment` while always keeping `defaultLight` — an unlit
  stage renders black, which reads as a broken sensor — and takes
  `keep_environment` for callers who want to keep it. Reloading an environment
  now replaces rather than stacking references.
- **`stop_simulation` silently kept the stepped pose** when called promptly
  after `step_simulation`. `_arm_reset_point` queues play/pause to give PhysX a
  restore point, but timeline transitions are tick-driven, so the point landed
  after `step()` returned. Deterministic on 6.0.1: a cube stepped from z=2.0
  stayed at z=-3.32 through stop. Arming now pumps once so the transition lands.
  The stepped result stays bit-identical. V6 only — V5 never had it. (#8 above
  covers the reset itself.)
- **`get_simulation_state` reported a Python repr as the version.** On 6.0
  `get_version()` returns an 8-tuple, not a string, so clients saw
  `"('6.0.1', 'rc.7', '6', ...)"` instead of `6.0.1-rc.7`. The same wrong
  assumption made adapter selection load V5 on a 6.0 runtime; that half had been
  fixed, this half had not. Both now read the duality from one place.
  (`adapters/version.py`) V6 only.
- **Both lidar tools were dead on 5.1.** `create_lidar` raised
  `got multiple values for keyword argument 'config'` (5.1's `LidarRtx` takes
  `config_file_name`), and `get_lidar_point_cloud` raised
  `'LidarRtx' object has no attribute 'get_point_cloud'` (5.1 exposes annotators
  plus `get_current_frame()`). The annotator must be attached *before*
  `initialize()` and the wrapper must be cached, exactly as cameras already are.
  V5 only — 6.0's lidar path was fixed earlier and left 5.1 behind.
- **Commands sent during startup failed with a raw AttributeError.** The socket
  accepts connections several seconds before Kit has a stage — measured on
  6.0.1 at t+6.8s versus t+14.5s, and an MCP client normally connects the moment
  the port opens. Every stage-dependent tool in that window returned
  `'NoneType' object has no attribute 'GetPrimAtPath'`, which reads as a broken
  server rather than one still starting. Dispatch now detects the pending stage
  and returns a message saying to retry. Both adapters.
- **Cameras could not be deleted.** `delete_object` on a camera returned
  success and the prim was still there a tick later, surviving `clear_scene`
  too, because an initialized Camera wrapper owns a render product, annotators
  and event subscriptions that keep its prim alive — dropping the cache entry is
  not enough, the wrapper has to be destroyed. Adapters now release the sensor
  before deleting, which also frees the render product that otherwise keeps
  rendering for the life of the Kit process. Measured on 5.1.0: camera prims
  1 -> 0 and render products 2 -> 1 on delete, where both previously stayed put.

  On 6.0 the wrappers offer no teardown at all — `RtxCamera` exposes only
  `reset_to_default_state`, `reset_xform_op_properties` and `valid`, and Isaac
  holds the instance internally — so the sensor re-created the prim on the tick
  *after* the delete, reappearing at the end of the parent's children with its
  render product still bound to it. Deactivating the prim before deleting breaks
  that binding and the delete holds. Verified on both: the camera was still
  present 4 s after a plain delete and absent 4 s after this one, with ordinary
  prims unaffected.
- **`get_lidar_point_cloud` returned a count but no point cloud.** The handler
  took `len()` of the decoded array and discarded it, so the tool could not
  produce the thing it is named after. Returning the whole sweep is not the
  answer either — 59k points is roughly 1.8 MB of JSON. It now returns a
  summary by default (`point_count`, `bounds`, `nearest` hit, and the frame the
  numbers are in), takes `max_points` to include a sample taken at an even
  stride across the sweep rather than the first N (which would be one slice of
  the scene), and takes `output_path` to write the full cloud as .npy for
  `numpy.load`. Measured: a summary with an 6-point sample is ~520 bytes.
  Both adapters.
- **The lidar's empty-read message gave the wrong advice two times out of
  three.** Every empty read said "call play_simulation", which is baffling when
  you already are and misleading when the sensor is simply looking at nothing —
  one placed at a robot's own origin returned 491 points in testing. The handler
  now checks the timeline and answers the case it is actually in: stopped ("the
  timeline is stopped — call play_simulation"), playing ("no completed sweep on
  this frame — retry; the sensor fills only when a rotation completes"), or
  indeterminate (covers both). Both adapters.
- **`create_camera` gained `target=`**, so a camera can be aimed at a point
  instead of by hand-computed euler angles. Cameras look down their local -Z and
  this extension's creation path gives them a non-identity `orient`, so aiming
  by arithmetic produced pictures of the sky even when the arithmetic was right
  — it took three attempts to frame one shot during testing. `target` needs a
  position (passed, or already on the prim), uses +Z as up, swaps the up axis
  when the view is parallel to it so straight-up and straight-down work, and
  leaves the orientation alone when eye and target coincide rather than
  authoring a garbage one. The response echoes `aimed_at` and the `rotation` it
  applied. Both adapters.
- **Newton was broken in two silent ways, and is now supported.** Nothing
  errored, so `step_simulation` returned a frozen world and `get_prim_info`
  reported the spawn pose forever.

  *Stepping.* No direct solver call advances Newton. Dropping a body from z=20
  for 60 steps, where free fall predicts 15.095: `SimulationManager.step`,
  `SimulationView.step(dt)` and `physx.update_simulation` all left it at
  20.0000 with zero velocity; only pumping the app tick ran the solver
  (15.0133, vz -9.8100). Stepping is now engine-aware — PhysX keeps
  `SimulationManager.step`, which is frame-exact, and Newton pumps. The Newton
  response carries `stepping: "approximate"`, because the app tick is about one
  step plus render jitter off exact and physics results are not
  frame-reproducible there. Pumping PhysX instead would have changed its
  answers too (-2.987 against -3.322 over the same fall), so it stays behind
  the engine branch.

  *Positions.* Newton writes simulated transforms to Fabric and never back to
  USD, so every USD read returned the spawn pose — a body falling at -9.81 m/s
  still read z=20.0. Rigid-body positions on Newton now come from the physics
  view, tagged `position_source: "physics"`. Measured after the fix: 14.8918
  after 60 steps and 2.8383 after a second of play, where both previously read
  20.0. PhysX keeps the USD path, which writes back and carries the prim's own
  local transform.

  Newton smoke test went from 11 checks and a socket timeout to 19/19.
- **`apply_material` leaked a raw USD C++ error** naming NVIDIA's build tree
  when a path did not exist. It validates both prims and names the offending
  one. Both adapters.

### Changed
- `scripts/smoke_test_v6.py` is now `scripts/smoke_test.py` and runs against
  either runtime, detecting the adapter from `simulation.get_state` and
  asserting what is true for each — V5 must *not* grow the V6-only reporting
  fields, so a misdetected adapter fails the run instead of passing quietly.
  Several of its checks had encoded contracts the code deliberately no longer
  has, or never had: two did `play` → `step` after that was made an error, and
  the reset check read a top-level `position` from `get_prim_info`, which has
  always nested it under `transform`.
- `clear_scene` gains `keep_environment`; `load_environment` returns
  `corrections` and `bounds`, and its `prim_path` now defaults to a named child
  of `/Environment` — read it from the response rather than assuming it.

### Fixed — Newton engine parity for the debug loop

- **A single Cone plus any robot permanently disabled physics on Newton.**
  Newton builds its model through `SolverMuJoCo`, whose `geom_type_mapping` has
  no entry for `GeoType.CONE` (9), so the conversion raises `KeyError:
  np.int32(9)`. Kit catches it, logs `[Newton] Initialization failed:
  np.int32(9)`, and latches `NewtonStage._init_failed = True` — a permanent
  latch, after which `initialize_newton()` returns early forever and every
  physics call dies with `Failed to create simulation view with backend
  'newton'`. Verified unrecoverable: deleting the cone, `clear_scene`, and
  rebuilding the `PhysicsScene` all still failed; only a Kit restart cleared it.
  `step`, `play`, `execute_script`, `get_joint_config` and `create_robot`'s
  joint report all went down together, so a session died on one `create_object`
  call.

  Measured on 6.0.1-rc.7, cold-booted per trial: cone + articulation bricks the
  session; a cone alone, a cylinder + articulation, and cone + articulation
  under PhysX are all fine — and 5.1 is unaffected (no Newton). The V6 adapter
  now refuses to initialise Newton physics while a cone and an articulation
  share the stage, naming the offending prims and pointing at
  Cylinder/Capsule/Sphere/Cube or the PhysX engine. Refusing *before*
  `initialize_physics()` is the whole point: the latch is never set, so
  deleting the cone recovers the session (verified) instead of requiring a
  restart. Articulation detection falls back to applied-schema names so a
  missing schema binding cannot quietly switch the guard off.

  This also explains three failing integration tests on Newton: the object
  tests leave a Cone on the stage, so everything after them inherited a dead
  simulator. `TestSimulationTools` now resets the stage it steps on.

- **Action Graphs ticked during `step_simulation` on Newton.** V6 advances
  Newton by pumping the app, which ticks OnPlaybackTick graphs — 30 ticks in a
  30-step call — so a ScriptNode could overwrite the joint targets being
  debugged, the exact hazard the PhysX path already guarded. The Newton pump
  now runs inside `_graphs_suspended` and reports `graphs_suspended`. Measured
  0 ticks during a step, with graphs restored for the following `play`.

### Known issues
- **Joint drives do not converge on Newton with PhysX-tuned gains.** Commanding
  the FR3 to j1=-0.4 / j3=-2.0 and stepping settles on PhysX in ~150 steps
  (-0.399 / -2.000) but oscillates indefinitely on Newton: measured across 25.5s
  of simulated time (1500 steps), j3 swung between -0.70 and -4.07 and never
  settled, with stiffness 60000 / damping 6000 as shipped with the asset. The
  stepping itself is sound — sim time advances 2.55s per 150 steps (60Hz) — so
  this is a solver/gain difference, not a stepping bug, and it is left
  untouched rather than papered over in the MCP layer. Re-tune the drive gains
  for Newton, or debug motion on PhysX.

  Beware a measurement trap here: joint reads taken when Newton is not actually
  simulating echo the commanded values back *exactly* (-0.400000004), which
  reads as perfect convergence. Real convergence carries residual error
  (PhysX: -0.399). Confirm `current_time` advanced before trusting a joint read.

- **Removing more than one RTX camera is unreliable, on both runtimes.** A
  single camera deletes cleanly and takes its render product with it (verified
  repeatedly, cold-booted). With several alive, only the first one or two go:
  measured 1 of 4 removed in one run and 2 of 4 in another with the same cadence
  and different orderings, so it is a race in Replicator's pipeline rather than
  an ordering rule. A camera that fails to delete is then stuck permanently —
  Replicator re-creates both the prim and its render product, the prim
  reappearing at the end of the parent's children — and repeated `clear_scene`
  calls make no further progress.

  Nothing reachable fixes it: neither wrapper exposes a working teardown
  (6.0's `RtxCamera` has none at all), deleting the bound render product first
  brings both back, and deactivating the prim is cosmetic — `capture_image`
  still succeeds on an inactive prim, so the sensor keeps rendering. Reuse a
  camera rather than creating several; a simulator restart clears the strays.
- Only one Isaac Sim instance can run at a time on a single GPU; a second
  concurrent instance caused device-lost crashes during testing.
- Newton physics is not frame-reproducible: stepping advances it through the app
  tick, so repeated runs of the same scene differ slightly. Use PhysX where
  exact stepping matters.

## [0.6.0] - 2026-06-13

### Added
- **Isaac Sim 6.0.0 support** — new `IsaacAdapterV6` built on `isaacsim.core.experimental.*` + `SimulationManager` + `isaacsim.sensors.experimental.rtx` + `isaacsim.asset.importer.urdf.URDFImporter`. Works under both the PhysX launcher (`isaac-sim.sh`) and the Newton launcher (`isaac-sim.newton.sh`).
- **Engine auto-detection** — `adapters/__init__.py:get_adapter()` reads `isaacsim.core.version.get_version()` and selects V5 or V6 by major version. V6 reads `SimulationManager.get_active_physics_engine()` at construction time.
- **`engine` and `isaacsim_version` fields on `get_simulation_state`** — MCP clients can see the active backend without poking at the runtime.

### Changed
- V6 URDF import uses `URDFImporter(URDFImporterConfig(...))` instead of the deprecated `URDFCreateImportConfig`/`URDFParseFile`/`URDFImportRobot` kit commands.
- V6 physics state reads route through `SimulationManager.get_physics_simulation_view()` (the `omni.physics.tensors` view), replacing the V5 direct call to `omni.physx.get_physx_interface().get_rigidbody_transformation()` (which is unavailable under the Newton kit).
- V6 sensor methods use `isaacsim.sensors.experimental.rtx.{RtxCamera,CameraSensor,Lidar,LidarSensor}` instead of the deprecated `isaacsim.sensors.camera.Camera` / `isaacsim.sensors.rtx.LidarRtx`.

### Notes
- 5.1.0 behavior unchanged — `IsaacAdapterV5` is untouched.
- Hot-reload script (`scripts/dev_mcp_server.sh`) now reloads `adapters.v6` alongside `adapters.v5`.

## [0.5.2] - 2026-04-07

### Fixed
- Code style: apply ruff formatting to v5 adapter, graphs handler, and scene handler

## [0.5.1] - 2026-04-06

### Added
- **`edit_action_graph` tool**: Modify attribute values and add connections on existing Action Graphs. Uses `og.Controller.set()` for ScriptNode `usePath`/`scriptPath` attributes (matching the pattern from `omni.graph.scriptnode` official tests). Auto-resets `state:omni_initialized` when script content or path changes to force ScriptNode reload
- **`script_file` parameter on `create_action_graph`**: One-step convenience for the common OnPlaybackTick → ScriptNode workflow. Automatically creates nodes, wires connections, and attaches the script file — replaces the previous two-step create + edit pattern
- **`prim_path` parameter on `create_robot`**: Explicit USD prim path control (e.g. `/World/Franka`) instead of name-based path derivation. Solves the common issue where robots are created at `/{Name}` but scripts expect `/World/{Name}`
- ScriptNode workflow documentation in MCP server instructions covering one-step (`script_file`) and two-step (`create` + `edit`) patterns, script reload via `edit_action_graph`, and `setup(db)`/`compute(db)` function requirements

### Changed
- `create_action_graph` docstring updated with `script_file` example and inline/file-based usage patterns
- `create_robot` docstring updated with `prim_path` parameter documentation
- Tool count updated to 42 across 9 categories

## [0.5.0] - 2026-04-06

### Added
- **`create_action_graph` tool**: Build OmniGraph Action Graphs programmatically (nodes, connections, attribute values) via `og.Controller.edit()` — no more raw `execute_script` calls for OnPlaybackTick → ScriptNode wiring
- **Drive config warnings**: `get_joint_config` and `create_robot` now return a `warnings` array when any joint has `stiffness=0` and `damping=0` (e.g. FR3 `finger_joint2` broken drive)
- **Dimensional data in responses**: `create_object` now returns `actual_size` [x, y, z] in meters and `bounding_box` (min/max world-space corners)
- **Prim size inspection**: `get_prim_info` returns `actual_size` for geometric prims (Cube, Sphere, Cylinder, Cone, Capsule)
- **Inline joint info**: `create_robot` now returns `joint_names` and `num_dof` in the response, eliminating the need for a follow-up `get_robot_info` call
- **Joint limits**: `get_robot_info` now returns `joint_limits` with type (revolute/prismatic), lower/upper limits, and units per joint
- **Comprehensive server instructions**: MCP `instructions` field now includes workflow guidance for scene setup, debug loop (step-and-observe), controller development, and tool priority
- `get_prim_actual_size` adapter method for computing prim dimensions from USD geometry attributes and scale

### Changed
- **Tool docstrings rewritten** with workflow guidance:
  - `step_simulation` promoted as the primary debug tool with typical debug loop example
  - `execute_script` reframed as escape hatch with explicit list of preferred alternatives
  - `reload_script` positioned as the controller loading workflow
  - `get_joint_config`, `get_physics_state`, `get_isaac_logs` marked as diagnostic tools with when-to-call guidance
  - `set_joint_positions`, `get_joint_positions` now document units (radians/meters)
  - `create_object` documents default primitive sizes and scale behavior
- Replaced `asset_creation_strategy` prompt with inline `instructions` covering MCP vs Script/Action Graph scope
- Updated package name and version in extension.toml
- Added new application icon and social badge image

### Fixed
- **Ground plane collision**: `create_physics_scene` now applies `UsdPhysics.CollisionAPI` to the ground plane — objects no longer fall through the floor
- **Stale `.pyc` in `reload_script`**: Dev script now clears bytecode cache before `importlib.reload()` for both extension and user modules, preventing stale code from loading
- **Orphaned subscriptions**: `reload_script` exec() mode now cleans up subscriptions from previous runs before re-executing
- Dev hot-reload script: bypass pybind11 `__setattr__` on `omni.ext.IExt` subclasses using `__dict__` assignment
- Dev hot-reload script: use `isinstance(obj, MCPExtension)` instead of fragile `hasattr` checks that matched wrong objects
- Dev hot-reload script: clear stale `.pyc` files before `importlib.reload()` to ensure fresh source is loaded
- Use `Usd.TimeCode.Default()` instead of non-existent `Gf.TimeCode(0)` in `get_prim_actual_size`
- World-space (not local-space) transform for bounding box computation
- Cylinder/Cone axis attribute respected when computing dimensions

## [0.4.1] - 2026-04-02

### Changed
- Added MCP registry metadata (`server.json`) for marketplace listing
- Fixed demo GIF URL in README to use absolute GitHub raw URL

## [0.4.0] - 2026-04-02

### Added
- **Observability tools**: `get_simulation_state`, `get_physics_state`, `get_joint_config`, `get_isaac_logs`, `reload_script`
- **Step-and-observe**: `observe` parameters on `step_simulation` for combined stepping and inspection (issue #8)
- `cwd` parameter and stdout/stderr capture for `execute_script`
- Franka pick-and-place demo scene and USD file
- Development wrapper for MCP server with hot-reloading support
- Environment discovery and loading tools
- Dynamic robot discovery from Isaac Sim asset server
- PyPI packaging via `pyproject.toml` — installable with `pip install isaacsim-mcp-server`
- Tag-triggered PyPI publish and GitHub Release CD pipeline
- Smithery registry manifest
- CI lint and format checks on PRs (ruff)
- Desktop launcher instructions and scripts
- Documentation for running multiple Isaac Sim instances with MCP

### Changed
- **Renamed package** from `isaac-sim-mcp` to `isaacsim-mcp-server` across all references
- Complete modular architecture rewrite:
  - Extracted `IsaacConnection` into dedicated connection module
  - Added adapter layer with base ABC and v5 implementation
  - Split into 8 handler modules with 31+ command handlers
  - Split into 8 MCP tool modules with 31+ tools
  - Rewrote `server.py` as slim entry point using modular tools
  - Rewrote `extension.py` as slim registry-based command router
  - Extracted socket server from `extension.py`
- Added type hints across all handler, adapter, and connection modules
- Migrated all imports from `omni.isaac.*` to `isaacsim.*` for Isaac Sim 5.1.0 compatibility
- Refreshed project documentation to reflect the current Isaac Sim `5.1.0`-focused architecture
- Reworked the README with a clearer quickstart, architecture overview, and example prompting workflow
- Updated build scripts to use installed `isaacsim-mcp-server` CLI
- Added MIT License to all source files; updated copyright headers for fork continuation
- Now documents `39` MCP tools across `8` categories

### Fixed
- Correct argument order in `set_channel_enabled` (issue #2 bug 1)
- Use PhysX velocity API for accurate runtime readings (issue #2 bug 2)
- Read runtime joint targets from articulation controller (issue #2 bug 3)
- Flatten `execute_script` and `reload_script` response structure (issue #2 bug 4)
- Use `add_message_consumer` API for Isaac Sim 5.1 log listener
- Compare log level enum by value for Isaac Sim 5.1 compatibility
- Use USD `RigidBodyAPI` velocity attrs instead of missing PhysX methods
- Initialize `SingleArticulation` before accessing controller APIs
- `scene.clear` now removes all user prims including root-level ones
- Fix transform precision conflict and URDF file validation
- Remove dead code and fix adapter bypass in handlers

### Tests
- Added 43 integration tests for all tool categories
- Updated structural tests for new observability methods

## [0.3.0] - 2025-04-22

### Added
- USD asset search integration with `search_3d_usd_by_text` tool
- Ability to search and load pre-existing 3D models from USD libraries
- Support for custom positioning and scaling of USD models
- Direct model transformation capabilities with the improved `transform` tool
- Enhanced scene management with multi-object placement

### Improved
- Scene object manipulation with precise positioning controls
- Asset loading performance and reliability
- Error handling for model search and placement
- Integration with existing physics scene management

### Technical Details
- Advanced USD model retrieval system
- Optimized asset loading pipeline
- Position and scale customization for USD models
- Better compatibility with Isaac Sim's native USD handling

## [0.2.1] - 2025-04-15

### Added
- Beaver3D integration for 3D model generation from text prompts and images
- Asynchronous model loading with asyncio support
- Task caching system to prevent duplicate model generation
- New MCP tools:
  - `generate_3d_from_text_or_image` for AI-powered 3D asset creation
  - `transform` for manipulating generated 3D models in the scene
- Texture and material binding for generated 3D models

### Improved
- Asynchronous command execution with `run_coroutine`
- Error handling and reporting for 3D generation tasks
- Performance optimizations for model loading

### Technical Details
- Integration with Beaver3D API for 3D generation
- Task monitoring with callback support
- Position and scale customization for generated models

## [0.1.0] - 2025-04-02

### Added
- Initial implementation of Isaac Sim MCP Extension
- Natural language control interface for Isaac Sim through MCP framework
- Core robot manipulation capabilities:
  - Dynamic placement and positioning of robots (Franka, G1, Go1, Jetbot)
  - Robot movement controls with position updates
  - Multi-robot grid creation (3x3 arrangement support)
- Advanced simulation features:
  - Quadruped robot walking simulation with waypoint navigation
  - Physics-based interactions between robots and environment
  - Custom lighting controls for better scene visualization
- Environment enrichment:
  - Various obstacle types: boxes, spheres, cylinders, cones
  - Wall creation for maze-like environments
  - Dynamic obstacle placement with customizable properties
- Development tools:
  - MCP server integration with Cursor AI
  - Debug interface accessible via local web server
  - Connection status verification with `get_scene_info`
- Documentation:
  - Installation instructions
  - Example prompts for common simulation scenarios
  - Configuration guidelines

### Technical Details
- Extension server running on localhost:8766
- Compatible with NVIDIA Isaac Sim 4.2.0
- Support for Python 3.9+
- MIT License for open development 
