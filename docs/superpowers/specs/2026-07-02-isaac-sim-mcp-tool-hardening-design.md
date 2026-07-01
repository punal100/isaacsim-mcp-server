# Isaac Sim MCP Tool Hardening — Design

**Date:** 2026-07-02
**Branch:** feat/isaac-sim-6.0.0-support
**Status:** Approved design (pending user spec review)

## Background

An agent driving the Isaac Sim MCP server through a full pick-and-place workflow
reported 8 concrete tool bugs / unexpected behaviours plus several gaps in the
server instructions. This design addresses all of them.

**Audience:** the consumer of these tools is an **AI engineering agent**, not a
human. Every enhancement is judged by whether it makes the *agent* more
effective — deterministic, self-correcting, no silent no-ops, minimal surface to
reason about. Features a human might like but an agent won't reason about (extra
metadata, speculative params) are cut, even when cheap to build.

**Guiding principle (user):** enhance existing MCP tools; do **not** add new
tools. Every additional tool costs schema surface and context tokens on every
agent call. See memory `prefer-enhancing-tools`.

**Cross-cutting theme (from the feedback):** silent failures and silent
behaviour are the worst offenders — a write that no-ops with no error, or a
timeline that free-runs under a "step" call, make root-causing slow because
there is no reliable signal. Fixes prefer **explicit errors and explicit state
in responses** over silent correction.

## Architecture context

Two layers, already established in the codebase:

- **MCP server** (`isaac_mcp/tools/*.py`) — tool definitions + docstrings; talks
  to the extension over a socket.
- **Isaac extension** (`isaac.sim.mcp_extension/.../`):
  - `handlers/*.py` — command dispatch, version-agnostic.
  - `adapters/{base,v5,v6}.py` — all Isaac-version-specific API calls.

Fixes land in whichever layer already owns the behaviour. No new tools, no new
modules.

---

## The fixes

### #1 — `step_simulation` must not race a free-running timeline

**Root cause (primary = prompt/instructions):** the debugging agent calls
`play_simulation` first out of habit, which puts the timeline into continuous
free-run; every subsequent `step_simulation(N)` then adds N frames *on top of*
wall-clock frames, so observations cannot be correlated to a frame index. The
tools actively teach this habit:
- `get_simulation_state` docstring says "verify the simulation is running before
  using step_simulation".
- `step_simulation`'s debug-loop docstring never says "no play needed first".

**Fix, in priority order:**

1. **Instructions / docstrings (primary).**
   - `step_simulation` docstring: state plainly that step is self-contained —
     it initialises physics on first call and operates on a **frozen** timeline;
     *"Do NOT call play_simulation before or during the debug loop. Use
     play_simulation only for a final continuous run."*
   - `get_simulation_state` docstring: remove "verify the simulation is running
     before using step_simulation".
   - MCP server instructions `### Debug Loop`: the debug loop is **step-only**;
     never `play` during it. `play` is reserved for the final continuous run /
     ScriptNode-driven demo.
2. **Code (fail-loud guard, defensive).** In `v5.step()` and `v6.step()`: if the
   timeline `is_playing()`, return
   `{"status": "error", "message": "Cannot step while the simulation is running.
   A free-running timeline is active — call pause_simulation or stop_simulation
   first. Do not call play_simulation during the debug loop; step_simulation is
   for a frozen timeline."}` **without stepping**. From a paused or stopped
   state, step proceeds: advance exactly N, leave the timeline paused. Include
   `"timeline_state": "paused"` and `"stepped": N` in the success result so N is
   always exact and correlatable. No silent auto-pause.

**Files:** `isaac_mcp/tools/simulation.py` (docstrings), the `## isaac-sim-dev`
instruction block, `adapters/v5.py` + `adapters/v6.py` (`step`).

### #2 — `create_action_graph` inline-script mode

**Root cause:** there is a one-parameter `script_file=` shortcut that
auto-creates `OnPlaybackTick → ScriptNode` and wires them, but no equivalent for
inline scripts. Passing `values=[{"attr":"ScriptNode.inputs:script",...}]`
without also creating the nodes fails (`node=None`), and hand-rolling the nodes
hits `Failed to wrap graph in node`. The docstring advertises an inline example
that does not work.

**Fix:** add an `inline_script=` parameter to `create_action_graph` mirroring
`script_file=`. When provided, auto-create `OnPlaybackTick → ScriptNode`, wire
`outputs:tick → inputs:execIn`, set `inputs:usePath=False` and
`inputs:script=<code>` (direct attribute set, same pattern as the scriptPath
path). Update the docstring so the advertised inline example matches what works;
remove the broken raw-`values` example. Document that `inline_script` is for
small/static graphs; `script_file=` is the **recommended** path for anything the
agent will iterate on, because it has the better reload story (see #3) —
editing an inline script requires `edit_action_graph`, editing a file + calling
`reload_script` "just works".

**Files:** `isaac_mcp/tools/graphs.py` (param + docstring),
`handlers/graphs.py` (`create_action_graph`).

### #3 — `reload_script` does not recompile a ScriptNode (no new tool)

**Root cause:** `reload_script` re-execs a standalone `.py` file into a fresh
namespace; it never touches the Action-Graph ScriptNode that references that
file, so editing the file on disk and calling `reload_script` does not recompile
the running node. The only working reload today is `edit_action_graph`
re-touching `scriptPath` (which already clears the ScriptNode caches).

**Fix (enhance existing tool, per principle):** make `reload_script` ScriptNode-aware.
After resolving the absolute file path, scan Action-Graph ScriptNodes for any
whose `inputs:scriptPath` matches it. For each match, force a recompile using the
existing robust path from `edit_action_graph`: reset `state:omni_initialized` to
False and clear the shared internal caches (`shared.use_path = None`,
`shared.script = None`). If at least one ScriptNode matched, report which nodes
were recompiled and skip the standalone re-exec. If none matched, fall back to
the current standalone re-exec behaviour. Update the docstring to describe both
modes.

Refactor the cache-clear logic in `edit_action_graph` into a small shared helper
in `handlers/graphs.py` so `reload_script`'s adapter path can reuse it (the
adapter calls into the handler helper, or the helper is placed where both reach
it).

**Files:** `adapters/v5.py` + `adapters/v6.py` (`reload_script`),
`handlers/graphs.py` (extract reusable recompile helper),
`isaac_mcp/tools/simulation.py` (docstring).

### #4 + #5 — `get_isaac_logs` run scoping and stdout capture

**Root cause:** the log buffer is global with no run boundaries, the listener is
registered lazily on the first `get_isaac_logs` call (so early diagnostics are
missed), only `WARN`/`ERROR` from `omni.log` are captured, and `print()`/stdout
is never captured. Entries cannot be correlated to a run or frame.

**Fix (trimmed to what an agent actually reasons about — "this run's logs, not
stale ones, including my print()"; run-id/sequence structured records and an
INFO `min_level` param were dropped as surface the agent won't use):**

1. **Register the listener at extension load**, not on first call, so early logs
   are captured. (Hook into the extension `on_startup`; keep the lazy
   `_ensure_log_listener` as an idempotent fallback.)
2. **Play-boundary scoping.** Subscribe to the timeline **Play** event and record
   the buffer index at that moment (a single boundary marker — no per-entry run
   id or sequence numbers). Add a `since_last_play` param to `get_isaac_logs`,
   **defaulting to True**, which returns only entries after the last Play
   boundary. This gives the requested "since last play" scoping without exposing
   metadata.
3. **Capture stdout.** Route captured stdout from `execute_script` and
   `reload_script` (already collected there) into the log buffer tagged
   `[PRINT]`, so `print()` surfaces in `get_isaac_logs` and not only in the
   direct tool response. ScriptNode `print()` is captured where reachable
   (documented as best-effort).
4. **Non-destructive default.** Flip `get_isaac_logs` to `clear=False` by
   default — "show me this run's logs" should not destroy the buffer. Explicit
   `clear=True` still empties it. Level filter stays WARN/ERROR (plus `[PRINT]`);
   no INFO firehose.

Response stays a list of strings (`logs`) with `log_count`, backward-compatible.

**Files:** `handlers/simulation.py` (`get_logs`, eager listener, play-boundary
index, `since_last_play` + `clear` defaults), `adapters/{v5,v6}.py` (feed
captured stdout into the buffer), `isaac_mcp/tools/simulation.py`
(`since_last_play` param + docstring), extension `on_startup` for eager
registration.

### #6 — `execute_script` can disturb a live Action Graph / ScriptNode

**Root cause:** running `execute_script` against an articulation that a live
ScriptNode controls can break the ScriptNode's control path with no error
surfaced. `execute_script` is the recommended escape hatch, so this is
surprising.

**Fix (documentation).** Add a caution to the `execute_script` docstring and the
MCP server instructions that distinguishes the two debug modes and the two
access types (avoiding the contradiction that you could `step` a running graph —
you cannot; see #1):

- **Two debug modes are separate.** The MCP debug loop
  (`set_joint_positions` + `step_simulation`) runs on a **frozen** timeline with
  **no graph running**. A ScriptNode/Action-Graph runs **within** frames and
  **requires Play** to tick — debug it with `play_simulation` + `get_isaac_logs`,
  **not** with `step_simulation` (stepping while playing is invalid and errors,
  per #1).
- **While a graph is running:** read-only diagnostics (`get_prim_info`,
  `get_physics_state`, `get_joint_positions`, `get_isaac_logs`) are safe.
  **Writing to or stepping** an articulation that a live ScriptNode controls can
  **silently break its control path** — `stop_simulation` (or pause the graph)
  **before** using `execute_script` / named write tools / `step_simulation` on
  the same articulation.

**Files:** `isaac_mcp/tools/simulation.py` (docstring), `## isaac-sim-dev`
instructions.

### #7 — `create_object` `scale=` footgun

**Root cause:** `size=` works as documented (absolute metres), but a raw
`scale=[0.4,0.4,0.3]` multiplies the primitive's **native** size, which is 2 m
for Cube/Sphere/Cylinder/Cone/Capsule (1 m for Plane), yielding
`[0.8,0.8,0.6]`. The docstring does not warn about this.

**Fix (documentation only — behaviour unchanged):** document that `scale=` is a
raw multiplier of the primitive's native size, list the native sizes (2 m for
Cube/Sphere/Cylinder/Cone/Capsule, 1 m for Plane), give the worked example
`scale=0.5 → 1 m`, and point users to `size=` for absolute metres.

**Files:** `isaac_mcp/tools/objects.py` (docstring).

### #8 — `stop_simulation` must reset to spawn state

**Root cause:** `stop()` calls `timeline.stop()` only; it performs no
`World`/`SimulationManager` reset, so articulation and rigid-body state is not
restored to the spawn pose on the next Play.

**Fix:** after `timeline.stop()`, restore pre-sim state — call
`World.instance().reset()` (v5) or the `SimulationManager` reset (v6), guarded so
a missing world degrades gracefully. Ensure the initial state is captured at
first `play()` (World captures this at reset/first play). Document in
`stop_simulation`: "resets articulations and rigid bodies to their spawn pose
(the state at first Play)."

**Files:** `adapters/v5.py` + `adapters/v6.py` (`stop`, and `play` if an explicit
initial-state capture is needed), `isaac_mcp/tools/simulation.py` (docstring).

### Server-instruction gaps (the "missing context" section)

Add to the `## isaac-sim-dev` MCP server instructions:
- **Stepping is authoritative and freezes the timeline;** never `play` during the
  MCP debug loop (see #1). `step` errors if the timeline is already playing.
- **Two separate debug modes:** MCP loop = `set_joint_positions` + `step` on a
  frozen timeline; ScriptNode/graph = `play` + `get_isaac_logs` (graphs tick only
  while playing and cannot be stepped). Do not mix them (see #6).
- **`stop` resets to spawn state** (see #8).
- **`get_isaac_logs` captures `carb.log_*`/`omni.log` plus captured stdout**
  tagged `[PRINT]`; plain `print()` outside captured contexts may not appear
  (see #5).
- **`execute_script` can disturb a live graph** (see #6).
- **ScriptNode physics contract:** what must be initialised before articulation
  writes take effect, and that such write failures are **silent** rather than
  raised (reinforce the existing WARMUP / initialize guidance).

---

## Testing

Extend the existing **offline** suites (no running Isaac Sim required):

- `tests/test_adapter_v6.py` — step returns an error when the timeline is
  playing and steps + stays paused otherwise; `stop` invokes a reset;
  `reload_script` matches ScriptNodes by `scriptPath` and recompiles them, else
  falls back to re-exec.
- `tests/test_handler_structure.py` — `get_logs` with `since_last_play=True`
  returns only entries after the last Play boundary; `clear=False` is the default
  and leaves the buffer intact; captured stdout appears tagged `[PRINT]`; the
  shared recompile helper is reused by both `edit_action_graph` and
  `reload_script`.
- `tests/test_tool_registration.py` — new params (`inline_script`,
  `since_last_play`) are registered with correct signatures and defaults;
  docstrings present.

Where Isaac APIs are needed, follow the existing pattern of mocking the adapter
/ `omni.*` surface (as the current v6 tests already do).

**Live verification** (manual, requires a running Isaac Sim) is added to
`scripts/smoke_test_v6.py`: real physics reset on stop, real ScriptNode
recompile after an on-disk edit, inline-script graph runs, and log run-scoping
across a stop/play cycle.

## Out of scope / non-goals

- No new MCP tools (explicit user constraint).
- No change to `create_object` `scale=` behaviour (docs only).
- No silent auto-correction of timeline state in `step` (fail-loud instead).
- No INFO-level logging surface (`min_level`) and no per-entry run-id/sequence
  metadata — trimmed as surface an agent won't reason about; `since_last_play`
  boundary scoping covers the real need.

## Rollout

Single branch off the current `feat/isaac-sim-6.0.0-support`. Docstring/
instruction changes and code changes ship together so the guidance and the
fail-loud guard reinforce each other. `CHANGELOG.md` updated. Version bump per
existing convention if released.
