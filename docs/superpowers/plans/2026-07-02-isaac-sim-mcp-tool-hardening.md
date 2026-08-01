# Isaac Sim MCP Tool Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix 8 reported Isaac Sim MCP tool bugs/gaps so an AI agent driving the tools gets deterministic stepping, fail-loud errors instead of silent no-ops, working inline graphs, ScriptNode-aware reload, run-scoped logs, and honest docstrings.

**Architecture:** Two layers. The **MCP server** (`isaac_mcp/tools/*.py`, `isaac_mcp/server.py`) defines tool signatures, docstrings, and the server-instruction block. The **Isaac extension** (`isaac.sim.mcp_extension/isaac_sim_mcp_extension/`) has version-agnostic `handlers/*.py` that dispatch to version-specific `adapters/{v5,v6}.py`. Each fix lands in whichever layer already owns the behaviour; no new tools and no new modules are added.

**Tech Stack:** Python 3.10, FastMCP (`mcp.server.fastmcp`), Isaac Sim 5.x/6.x runtime APIs (`omni.timeline`, `omni.graph.core`, `omni.log`, `isaacsim.core.*`), pytest with `ast`-based structural tests and `sys.modules` stub injection (`tests/conftest.py`).

## Global Constraints

- **Audience is an AI agent, not a human** — judge every change by agent effectiveness: deterministic, self-correcting, no silent no-ops, minimal surface. Cut metadata/params an agent won't reason about. (See spec + memory `tools-target-ai-agent`.)
- **No new MCP tools** — enhance existing tools only. (memory `prefer-enhancing-tools`.)
- **No silent auto-correction** — surface an explicit error instead of silently fixing state.
- **License header** — every new source file starts with the MIT header block copied verbatim from an existing sibling file (lines 1–22 of any `isaac_mcp/tools/*.py`).
- **Test strategy (matches the repo):** pure-Python logic gets real pytest unit tests; code that requires live `omni`/`og`/`isaacsim` APIs gets `ast`-based presence tests (the pattern already used in `tests/test_handler_structure.py` / `tests/test_tool_registration.py`) plus a line added to the manual `scripts/smoke_test_v6.py`. `tests/conftest.py` already stubs `carb`, `omni.*`, `pxr`, `numpy`.
- **Run tests with:** `python -m pytest tests/ -q` from the repo root.
- **Commit after every task.** Branch is `feat/isaac-sim-6.0.0-support` (already checked out).

Spec: `docs/superpowers/specs/2026-07-02-isaac-sim-mcp-tool-hardening-design.md`.

---

### Task 1: #7 — `create_object` scale docstring (docs-only)

**Files:**
- Modify: `isaac_mcp/tools/objects.py:48-68` (docstring)
- Test: `tests/test_tool_docstrings.py` (create)

**Interfaces:**
- Consumes: nothing.
- Produces: nothing consumed by later tasks. Establishes `tests/test_tool_docstrings.py` with a `_read_tool_source(filename)` helper that later tasks reuse.

- [ ] **Step 1: Write the failing test**

Create `tests/test_tool_docstrings.py`:

```python
# (copy the MIT header block, lines 1-22, from isaac_mcp/tools/objects.py)

"""Substring checks on MCP tool docstrings and the server instruction block."""

import os

TOOLS_DIR = os.path.join(os.path.dirname(__file__), "..", "isaac_mcp", "tools")
SERVER_PY = os.path.join(os.path.dirname(__file__), "..", "isaac_mcp", "server.py")


def _read_tool_source(filename):
    with open(os.path.join(TOOLS_DIR, filename)) as f:
        return f.read()


def _read_server_source():
    with open(SERVER_PY) as f:
        return f.read()


def test_create_object_documents_scale_multiplier():
    src = _read_tool_source("objects.py")
    # scale= is a raw multiplier of the primitive's native size
    assert "native size" in src
    assert "2 m" in src or "2m" in src  # native size of Cube/Sphere/etc
    assert "scale=0.5" in src  # worked example -> 1 m
    assert "size=" in src  # steer to size= for absolute meters
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_tool_docstrings.py::test_create_object_documents_scale_multiplier -v`
Expected: FAIL (`assert "native size" in src` — text not present yet).

- [ ] **Step 3: Update the docstring**

In `isaac_mcp/tools/objects.py`, replace the docstring body (lines 48-68) so the `scale`/`size` guidance reads:

```python
        """Create a primitive object (Cube, Sphere, Cylinder, Cone, Capsule, Plane).

        Prefer `size` for absolute sizing: `size` is the target in METERS
        (default 1.0), so `size=0.3` gives a 0.3 m object regardless of type.

        `scale` is a RAW MULTIPLIER of the primitive's NATIVE size, not meters.
        Native sizes: Cube/Sphere/Cylinder/Cone/Capsule = 2 m, Plane = 1 m.
        So `scale=0.5` on a Cube -> 1 m, and `scale=[0.4,0.4,0.3]` -> a
        0.8 x 0.8 x 0.6 m box (0.4 * 2 m), which surprises callers who expect
        0.4 m. Use `scale` only for deliberate non-uniform shaping; otherwise
        use `size`. If both are given, `scale` wins and `size` is ignored.

        Returns prim_path, actual_size [x, y, z] in meters, and bounding_box
        (min/max corners in world coordinates) so you can accurately place
        other objects relative to this one.

        Args:
            object_type: Type of primitive — Cube, Sphere, Cylinder, Cone, Capsule, or Plane.
            position: [x, y, z] world position.
            rotation: [rx, ry, rz] rotation in degrees.
            scale: [sx, sy, sz] RAW multiplier of the native size (2 m for most
                prims, 1 m for Plane). NOT meters. Overrides `size`.
            size: Target size in METERS (default 1.0). Absolute; independent of
                the primitive's native size. Ignored if `scale` is provided.
            color: [r, g, b] color values (0-1).
            physics_enabled: Enable physics on this object.
            prim_path: Custom prim path. Auto-generated if not provided.
        """
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_tool_docstrings.py::test_create_object_documents_scale_multiplier -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_tool_docstrings.py isaac_mcp/tools/objects.py
git commit -m "docs: clarify create_object scale= is a raw native-size multiplier (#7)"
```

---

### Task 2: #1 — `step_simulation` fail-loud guard + anti-`play` guidance

**Files:**
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py:71-81` (`step` handler)
- Modify: `isaac_mcp/tools/simulation.py:67-99` (`step_simulation` docstring), `145-155` (`get_simulation_state` docstring)
- Modify: `isaac_mcp/server.py:73-78` (Debug Loop / Controller Development instructions)
- Test: `tests/test_handler_simulation.py` (create), `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: `adapter.get_simulation_state()` → `dict` with key `"timeline_state"` in `{"playing","paused","stopped"}` (already implemented in both adapters).
- Produces: `handlers.simulation.step(adapter, num_steps, observe_prims, observe_joints)` returns `{"status":"error", "message": <free-run message>}` when playing, else `{"status":"success", "message": ..., "timeline_state": <state>, ...adapter result}`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_handler_simulation.py`:

```python
# (copy the MIT header block, lines 1-22, from isaac_mcp/tools/objects.py)

"""Behavioural unit tests for the simulation command handlers (mock adapter)."""

from unittest.mock import MagicMock

from isaac_sim_mcp_extension.handlers import simulation as sim


def test_step_refuses_when_timeline_playing():
    adapter = MagicMock()
    adapter.get_simulation_state.return_value = {"timeline_state": "playing"}

    result = sim.step(adapter, num_steps=5)

    assert result["status"] == "error"
    assert "running" in result["message"].lower()
    adapter.step.assert_not_called()


def test_step_runs_when_paused_and_reports_state():
    adapter = MagicMock()
    adapter.get_simulation_state.return_value = {"timeline_state": "paused"}
    adapter.step.return_value = {"stepped": 3}

    result = sim.step(adapter, num_steps=3)

    assert result["status"] == "success"
    assert result["timeline_state"] == "paused"
    assert result["stepped"] == 3
    adapter.step.assert_called_once()


def test_step_runs_when_stopped():
    adapter = MagicMock()
    adapter.get_simulation_state.return_value = {"timeline_state": "stopped"}
    adapter.step.return_value = {"stepped": 1}

    result = sim.step(adapter, num_steps=1)

    assert result["status"] == "success"
    adapter.step.assert_called_once()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_handler_simulation.py -v`
Expected: FAIL — `test_step_refuses_when_timeline_playing` fails because the current handler always calls `adapter.step`.

- [ ] **Step 3: Implement the guard in the handler**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py`, replace the `step` function (lines 71-81) with:

```python
def step(
    adapter: IsaacAdapterBase,
    num_steps: int = 1,
    observe_prims: Optional[Sequence[str]] = None,
    observe_joints: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
    try:
        # Fail loud: stepping is only valid on a frozen (paused/stopped)
        # timeline. If a free run is active, N frames cannot be counted
        # exactly, so refuse rather than silently race the play loop.
        state = adapter.get_simulation_state()
        timeline_state = state.get("timeline_state") if isinstance(state, dict) else None
        if timeline_state == "playing":
            return {
                "status": "error",
                "message": (
                    "Cannot step while the simulation is running. A free-running "
                    "timeline is active — call pause_simulation or stop_simulation "
                    "first. Do not call play_simulation during the debug loop; "
                    "step_simulation is for a frozen timeline."
                ),
            }
        result = adapter.step(num_steps=num_steps, observe_prims=observe_prims, observe_joints=observe_joints)
        return {
            "status": "success",
            "message": f"Stepped {num_steps} frames",
            "timeline_state": timeline_state,
            **result,
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_handler_simulation.py -v`
Expected: PASS (all three).

- [ ] **Step 5: Write the docstring/instruction test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_step_simulation_docstring_forbids_play_first():
    src = _read_tool_source("simulation.py")
    assert "Do NOT call play_simulation" in src
    assert "frozen" in src


def test_get_simulation_state_drops_verify_running_claim():
    src = _read_tool_source("simulation.py")
    assert "verify the simulation is running before" not in src


def test_server_instructions_debug_loop_is_step_only():
    src = _read_server_source()
    assert "step-only" in src
    assert "never play" in src.lower() or "do not call play_simulation" in src.lower()
```

- [ ] **Step 6: Run to verify these fail**

Run: `python -m pytest tests/test_tool_docstrings.py -v -k "play_first or verify_running or step_only"`
Expected: FAIL (text not present yet).

- [ ] **Step 7: Update the docstrings and server instructions**

In `isaac_mcp/tools/simulation.py`, replace the `step_simulation` docstring (lines 71-88) with:

```python
        """Advance the simulation by exactly N physics frames on a FROZEN timeline.

        step is self-contained: it initialises physics on first call and operates
        on a paused/stopped timeline, so N is always exact and observations
        correlate to a known frame count.

        Do NOT call play_simulation before or during the debug loop. If the
        timeline is already playing, step returns an error (a free run cannot be
        counted frame-by-frame). Use play_simulation ONLY for a final continuous
        run / ScriptNode-driven demo, never for debugging.

        Typical debug loop (no play):
          1. set_joint_positions to command the robot
          2. step_simulation with observe_prims and observe_joints
          3. get_joint_config if drives are not tracking correctly
          4. get_physics_state if objects are not behaving as expected
          5. Adjust and repeat

        Args:
            num_steps: Number of simulation frames to step.
            observe_prims: List of prim paths to observe (returns position + velocity).
            observe_joints: List of articulation prim paths to observe (returns joint positions).
        """
```

In the same file replace the `get_simulation_state` docstring (lines 147-149) with:

```python
        """Get the current simulation state: timeline status (playing/stopped/paused),
        simulation time, and physics dt. step_simulation does NOT require a running
        timeline — do not play just to step."""
```

In `isaac_mcp/server.py`, replace the Debug Loop + Controller Development block (lines 73-78) with:

```python
### Debug Loop (step-only — never play)
The debug loop is step-only: set_joint_positions + step_simulation with
observe_prims/observe_joints on a FROZEN timeline. Do NOT call play_simulation
while debugging — step errors if the timeline is already playing. If issues:
get_joint_config, get_physics_state, get_isaac_logs.
play_simulation is ONLY for a final continuous run / ScriptNode demo.
Two separate debug modes: MCP loop = step on a frozen timeline (no graph);
ScriptNode/Action-Graph = play + get_isaac_logs (graphs tick only while playing
and cannot be stepped). Do not mix them.

### Controller Development
Write .py file → reload_script → step_simulation to debug → edit & reload →
play_simulation only for the final continuous run.
```

- [ ] **Step 8: Run the full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add tests/test_handler_simulation.py tests/test_tool_docstrings.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py \
        isaac_mcp/tools/simulation.py isaac_mcp/server.py
git commit -m "fix: step_simulation fails loud on a running timeline + step-only debug guidance (#1)"
```

---

### Task 3: #8 — `stop_simulation` resets to spawn state

**Files:**
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py:918-921` (`stop`)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py:878-881` (`stop`)
- Modify: `isaac_mcp/tools/simulation.py:57-65` (`stop_simulation` docstring)
- Modify: `scripts/smoke_test_v6.py` (add live check note)
- Test: `tests/test_adapter_stop_reset.py` (create), `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: nothing new.
- Produces: `IsaacAdapterV5.stop()` / `IsaacAdapterV6.stop()` call `timeline.stop()` then a physics reset, guarded so a missing world/SimulationManager degrades gracefully.

- [ ] **Step 1: Write the failing test (AST presence — reset is live-only)**

Create `tests/test_adapter_stop_reset.py`:

```python
# (copy the MIT header block, lines 1-22, from isaac_mcp/tools/objects.py)

"""Verify stop() performs a physics reset (AST) — live behaviour is in smoke_test_v6."""

import ast
import os

ADAPTERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
)


def _stop_body_src(filename):
    with open(os.path.join(ADAPTERS, filename)) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "stop":
            return ast.get_source_segment(open(os.path.join(ADAPTERS, filename)).read(), node)
    return ""


def test_v6_stop_resets_physics():
    src = _stop_body_src("v6.py")
    assert "reset" in src.lower()
    assert "stop()" in src  # still stops the timeline first


def test_v5_stop_resets_physics():
    src = _stop_body_src("v5.py")
    assert "reset" in src.lower()
    assert "stop()" in src
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_adapter_stop_reset.py -v`
Expected: FAIL (`reset` not in current `stop` bodies).

- [ ] **Step 3: Implement reset in v6**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py`, replace `stop` (lines 918-921) with:

```python
    def stop(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
        # Restore articulations / rigid bodies to their spawn pose (the state
        # captured at first Play), matching the Isaac UI Stop button. Guarded so
        # a scene with no initialised physics still stops cleanly.
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            SimulationManager.reset_simulation()
        except Exception:
            pass
```

- [ ] **Step 4: Implement reset in v5**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py`, replace `stop` (lines 878-881) with:

```python
    def stop(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
        # Restore articulations / rigid bodies to their spawn pose (the state
        # captured at first Play), matching the Isaac UI Stop button. Guarded so
        # a scene with no World instance still stops cleanly.
        try:
            from isaacsim.core.api import World

            world = World.instance()
            if world is not None:
                world.reset()
        except Exception:
            pass
```

- [ ] **Step 5: Run test to verify it passes**

Run: `python -m pytest tests/test_adapter_stop_reset.py -v`
Expected: PASS.

- [ ] **Step 6: Update the docstring + add a docstring test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_stop_simulation_documents_reset():
    src = _read_tool_source("simulation.py")
    assert "spawn pose" in src
    assert "reset" in src.lower()
```

In `isaac_mcp/tools/simulation.py`, replace the `stop_simulation` docstring (line 59) with:

```python
        """Stop the physics simulation and reset to spawn state.

        Resets articulations and rigid bodies to their spawn pose (the state
        captured at first Play), like the Isaac UI Stop button. Call this to
        return the scene to a clean starting point before another run."""
```

- [ ] **Step 7: Add the live check to the smoke test**

Append a short block to `scripts/smoke_test_v6.py` that: creates a cube above the ground, plays, steps until it falls, calls `stop_simulation`, then asserts the cube's world Z is back at its spawn value. Follow the existing socket-call style already in that file. (Manual — not run in CI.)

- [ ] **Step 8: Run the full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add tests/test_adapter_stop_reset.py tests/test_tool_docstrings.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py \
        isaac_mcp/tools/simulation.py scripts/smoke_test_v6.py
git commit -m "fix: stop_simulation resets scene to spawn state (#8)"
```

---

### Task 4: #2 — `create_action_graph` inline_script shortcut

**Files:**
- Modify: `isaac_mcp/tools/graphs.py:37-95` (add `inline_script` param + fix docstring)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py:38-135` (handle `inline_script`)
- Test: `tests/test_handler_graphs.py` (create), `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: nothing new.
- Produces: `create_action_graph(..., inline_script: Optional[str])` — when set, the handler auto-creates `OnPlaybackTick → ScriptNode`, wires `outputs:tick → inputs:execIn`, and sets `inputs:usePath=False` + `inputs:script=<code>`.

- [ ] **Step 1: Write the failing test (AST — og is live-only)**

Create `tests/test_handler_graphs.py`:

```python
# (copy the MIT header block, lines 1-22, from isaac_mcp/tools/objects.py)

"""Structural tests for the action-graph handler inline_script path."""

import ast
import os

HANDLERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "handlers",
)


def _handler_src():
    with open(os.path.join(HANDLERS, "graphs.py")) as f:
        return f.read()


def test_create_action_graph_accepts_inline_script_param():
    tree = ast.parse(_handler_src())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "create_action_graph":
            arg_names = {a.arg for a in node.args.args} | {a.arg for a in node.args.kwonlyargs}
            assert "inline_script" in arg_names
            return
    raise AssertionError("create_action_graph not found")


def test_inline_script_sets_script_and_disables_usepath():
    src = _handler_src()
    # inline path builds the same OnPlaybackTick -> ScriptNode pair and sets script inline
    assert "inline_script" in src
    assert "inputs:script" in src
    assert "inputs:usePath" in src
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_handler_graphs.py -v`
Expected: FAIL (`inline_script` param absent).

- [ ] **Step 3: Add inline_script handling in the handler**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py`, change the `create_action_graph` signature (line 38-46) to add the param after `script_file`:

```python
def create_action_graph(
    adapter: IsaacAdapterBase,
    graph_path: str = "/World/ActionGraph",
    nodes: Optional[List[Dict[str, str]]] = None,
    connections: Optional[List[List[str]]] = None,
    values: Optional[List[Dict[str, object]]] = None,
    evaluator: str = "push",
    script_file: Optional[str] = None,
    inline_script: Optional[str] = None,
) -> Dict[str, Any]:
```

Replace the `script_file` shortcut block (lines 60-67) with a combined shortcut that handles both file and inline:

```python
        # ── shortcut: create standard OnPlaybackTick -> ScriptNode graph ─
        if script_file is not None or inline_script is not None:
            nodes = [
                {"path": "OnPlaybackTick", "type": "omni.graph.action.OnPlaybackTick"},
                {"path": "ScriptNode", "type": "omni.graph.scriptnode.ScriptNode"},
            ]
            connections = [["OnPlaybackTick.outputs:tick", "ScriptNode.inputs:execIn"]]
            values = None  # script/scriptPath set via direct attribute set below
```

Then replace the post-edit attach block (lines 118-127) with one that attaches either the file path or the inline script:

```python
        # ── attach script via direct attribute set ─────────────────
        if (script_file is not None or inline_script is not None) and graph is not None:
            script_node = graph.get_node(f"{graph_path}/ScriptNode")
            if script_node is not None and script_node.is_valid():
                use_path_attr = script_node.get_attribute("inputs:usePath")
                if script_file is not None:
                    script_path_attr = script_node.get_attribute("inputs:scriptPath")
                    if use_path_attr is not None and use_path_attr.is_valid():
                        og.Controller.set(use_path_attr, True)
                    if script_path_attr is not None and script_path_attr.is_valid():
                        og.Controller.set(script_path_attr, script_file)
                else:  # inline_script
                    script_attr = script_node.get_attribute("inputs:script")
                    if use_path_attr is not None and use_path_attr.is_valid():
                        og.Controller.set(use_path_attr, False)
                    if script_attr is not None and script_attr.is_valid():
                        og.Controller.set(script_attr, inline_script)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_handler_graphs.py -v`
Expected: PASS.

- [ ] **Step 5: Add the tool param + docstring test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_create_action_graph_has_inline_script_param():
    import ast

    src = _read_tool_source("graphs.py")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "create_action_graph":
            arg_names = {a.arg for a in node.args.args} | {a.arg for a in node.args.kwonlyargs}
            assert "inline_script" in arg_names
            assert "script_file" in src and "recommended" in src.lower()
            return
    raise AssertionError("create_action_graph tool not found")
```

- [ ] **Step 6: Run to verify it fails**

Run: `python -m pytest tests/test_tool_docstrings.py::test_create_action_graph_has_inline_script_param -v`
Expected: FAIL.

- [ ] **Step 7: Add the tool param + fix the docstring**

In `isaac_mcp/tools/graphs.py`, add `inline_script: Optional[str] = None` to the `create_action_graph` signature (after `script_file`, line 44). Replace the two `Example` blocks (lines 68-78) with:

```python
            inline_script: Convenience shortcut — inline Python (must define
                setup(db)/compute(db)). Auto-creates OnPlaybackTick → ScriptNode,
                wires them, and sets the script inline (usePath=False). For small,
                static graphs. For anything you will iterate on, prefer
                script_file — it has the better reload story (edit the file +
                reload_script "just works"; inline edits need edit_action_graph).

        Example (inline script — one-step):
            create_action_graph(
                inline_script="def setup(db): pass\\ndef compute(db): return True"
            )

        Example (script file — one-step, recommended for iteration):
            create_action_graph(
                script_file="/path/to/controller.py"
            )
        """
```

Update the param-forwarding block (lines 82-91) so `inline_script` is forwarded:

```python
            conn = get_connection()
            params: Dict[str, object] = {"graph_path": graph_path, "evaluator": evaluator}
            if script_file is not None:
                params["script_file"] = script_file
            elif inline_script is not None:
                params["inline_script"] = inline_script
            else:
                if nodes is not None:
                    params["nodes"] = nodes
                if connections is not None:
                    params["connections"] = connections
                if values is not None:
                    params["values"] = values
```

- [ ] **Step 8: Run to verify it passes + full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add tests/test_handler_graphs.py tests/test_tool_docstrings.py \
        isaac_mcp/tools/graphs.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py
git commit -m "feat: create_action_graph inline_script one-step shortcut (#2)"
```

---

### Task 5: #3 — `reload_script` recompiles ScriptNodes (shared helper, no new tool)

**Files:**
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py` (extract `force_recompile_scriptnode(graph, node)` helper from the inline block at lines 259-282; call it there)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py:1072-1142` (`reload_script`: scan ScriptNodes by scriptPath, recompile matches)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py:1026-…` (`reload_script`: same scan)
- Modify: `isaac_mcp/tools/simulation.py:226-251` (`reload_script` docstring)
- Test: `tests/test_handler_graphs.py` (extend), `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: `og.get_graph_by_path`, `graph.get_node`, ScriptNode attributes (live).
- Produces: `handlers.graphs.force_recompile_scriptnode(graph, node) -> None` — resets `state:omni_initialized` and clears `shared.use_path`/`shared.script`. Both `edit_action_graph` and adapter `reload_script` call it.

- [ ] **Step 1: Write the failing test (AST)**

Add to `tests/test_handler_graphs.py`:

```python
def test_force_recompile_helper_exists_and_is_reused():
    tree = ast.parse(_handler_src())
    func_names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "force_recompile_scriptnode" in func_names
    # edit_action_graph delegates to the shared helper rather than inlining it
    assert _handler_src().count("force_recompile_scriptnode(") >= 2


def test_reload_script_scans_scriptnodes_by_scriptpath():
    for fname in ("v6.py", "v5.py"):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "isaac.sim.mcp_extension",
            "isaac_sim_mcp_extension",
            "adapters",
            fname,
        )
        with open(path) as f:
            src = f.read()
        assert "inputs:scriptPath" in src  # reload matches nodes by their file
        assert "force_recompile_scriptnode" in src  # and recompiles them
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_handler_graphs.py -v -k "force_recompile or scans_scriptnodes"`
Expected: FAIL.

- [ ] **Step 3: Extract the shared helper in handlers/graphs.py**

Add this module-level function to `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py` (below `register`, above `create_action_graph`):

```python
def force_recompile_scriptnode(graph, node) -> None:
    """Force a ScriptNode to re-read and recompile its script.

    Resets the USD state attribute and clears the ScriptNode's internal shared
    caches so compute() detects a change even if a racing graph evaluation
    re-set omni_initialized. Safe to call when the scriptnode extension is not
    loaded (falls back to the attribute reset only).
    """
    import omni.graph.core as og

    attr = node.get_attribute("state:omni_initialized")
    if attr is not None and attr.is_valid():
        og.Controller.set(attr, False)
    try:
        from omni.graph.scriptnode.ogn.OgnScriptNodeDatabase import OgnScriptNodeDatabase

        shared = OgnScriptNodeDatabase.shared_internal_state(node)
        shared.use_path = None
        shared.script = None
    except Exception:
        pass
```

In `edit_action_graph`, replace the inline reset body (lines 262-282, the `if node is not None and node.is_valid():` block through `changes_made.append(...)`) with a call to the helper:

```python
                        if node is not None and node.is_valid():
                            force_recompile_scriptnode(graph, node)
                            changes_made.append(f"auto-reset state:omni_initialized on {node_path}")
```

- [ ] **Step 4: Add ScriptNode scan to v6.reload_script**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py`, at the start of `reload_script` (right after computing `abs_path`, before the standalone-exec fallback), insert a scan that recompiles any matching ScriptNode and returns early:

```python
        abs_path = os.path.abspath(file_path)

        # ScriptNode-aware reload: if any Action-Graph ScriptNode references this
        # file via inputs:scriptPath, force it to recompile (the standalone
        # re-exec below would not touch the running graph node).
        recompiled = _recompile_scriptnodes_for_file(abs_path)
        if recompiled:
            return {
                "status": "success",
                "message": f"Recompiled ScriptNode(s) referencing {os.path.basename(file_path)}",
                "recompiled_nodes": recompiled,
            }
```

Add this module-level helper near the top of `v6.py` (after the imports / before the class, or as a module function):

```python
def _recompile_scriptnodes_for_file(abs_path: str) -> list:
    """Recompile every Action-Graph ScriptNode whose scriptPath matches abs_path.

    Returns the list of recompiled node paths (empty if none matched).
    """
    import os

    try:
        import omni.graph.core as og

        from ..handlers.graphs import force_recompile_scriptnode
    except Exception:
        return []

    recompiled = []
    try:
        graphs = og.get_all_graphs() if hasattr(og, "get_all_graphs") else []
    except Exception:
        graphs = []
    for graph in graphs:
        try:
            for node in graph.get_nodes():
                attr = node.get_attribute("inputs:scriptPath")
                if attr is None or not attr.is_valid():
                    continue
                val = attr.get()
                if val and os.path.abspath(str(val)) == abs_path:
                    force_recompile_scriptnode(graph, node)
                    recompiled.append(node.get_prim_path())
        except Exception:
            continue
    return recompiled
```

- [ ] **Step 5: Add the same scan to v5.reload_script**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py`, add the identical `_recompile_scriptnodes_for_file` module helper and the same early-return scan at the start of `reload_script` (after `abs_path` is computed). The helper body is identical to v6's (it only depends on `og` + the handler helper).

- [ ] **Step 6: Run the AST tests to verify they pass**

Run: `python -m pytest tests/test_handler_graphs.py -v -k "force_recompile or scans_scriptnodes"`
Expected: PASS.

- [ ] **Step 7: Update the reload_script docstring + test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_reload_script_documents_scriptnode_mode():
    src = _read_tool_source("simulation.py")
    assert "ScriptNode" in src
    assert "recompile" in src.lower()
```

In `isaac_mcp/tools/simulation.py`, replace the `reload_script` docstring (lines 228-242) with:

```python
        """Reload a Python controller from a file on disk.

        Two modes, chosen automatically:
        - If any Action-Graph ScriptNode references this file (inputs:scriptPath),
          those ScriptNodes are force-recompiled so your on-disk edits take effect
          on the running graph. This is how you iterate on a ScriptNode controller.
        - Otherwise the file is (re-)executed as a standalone controller, the way
          you would use execute_script for code longer than ~20 lines.

        Workflow:
          1. Write the controller as a .py file (attach via create_action_graph
             script_file=... for ScriptNode use)
          2. reload_script to load / recompile it
          3. step_simulation to debug (frozen timeline) or play for a ScriptNode demo
          4. Edit the file and reload_script again to iterate

        The file's directory is auto-added to sys.path.

        Args:
            file_path: Path to the Python file on disk.
            module_name: Optional module name to reload (e.g. 'my_controller').
        """
```

- [ ] **Step 8: Run the full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Add a live check to the smoke test**

In `scripts/smoke_test_v6.py`, add a block that: writes a ScriptNode controller file, `create_action_graph(script_file=...)`, plays, edits the file on disk, calls `reload_script(file)`, and asserts the response contains `recompiled_nodes`. (Manual.)

- [ ] **Step 10: Commit**

```bash
git add tests/test_handler_graphs.py tests/test_tool_docstrings.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/graphs.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py \
        isaac_mcp/tools/simulation.py scripts/smoke_test_v6.py
git commit -m "fix: reload_script recompiles matching Action-Graph ScriptNodes (#3)"
```

---

### Task 6: #4 + #5 — `get_isaac_logs` run scoping, eager listener, print capture

**Files:**
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py:150-193` (log buffer, boundary, `_select_logs` helper, `append_log`, `get_logs`)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py:1033-1142` and `v5.py` (feed captured stdout into the log buffer)
- Modify: `isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py:53-61` (`on_startup`: eager listener + Play boundary subscription)
- Modify: `isaac_mcp/tools/simulation.py:126-143` (`get_isaac_logs` params/docstring: `since_last_play`, `clear` default)
- Test: `tests/test_log_buffer.py` (create), `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: nothing new.
- Produces, in `handlers/simulation.py`:
  - `append_log(entry: str) -> None` — append to `_log_buffer`, trimming to `_MAX_LOG_BUFFER`.
  - `mark_play_boundary() -> None` — set `_play_boundary = len(_log_buffer)`.
  - `_select_logs(buffer: list, boundary: int, since_last_play: bool, count: int) -> list` — pure selector.
  - `get_logs(adapter, clear=False, count=100, since_last_play=True) -> dict`.

- [ ] **Step 1: Write the failing test (pure logic)**

Create `tests/test_log_buffer.py`:

```python
# (copy the MIT header block, lines 1-22, from isaac_mcp/tools/objects.py)

"""Unit tests for the get_isaac_logs buffer selection logic."""

from isaac_sim_mcp_extension.handlers import simulation as sim


def test_select_logs_since_last_play_filters_to_current_run():
    buf = ["a", "b", "c", "d"]
    # Play happened after 'b' -> boundary index 2
    assert sim._select_logs(buf, boundary=2, since_last_play=True, count=100) == ["c", "d"]


def test_select_logs_all_when_not_scoped():
    buf = ["a", "b", "c", "d"]
    assert sim._select_logs(buf, boundary=2, since_last_play=False, count=100) == ["a", "b", "c", "d"]


def test_select_logs_respects_count():
    buf = ["a", "b", "c", "d"]
    assert sim._select_logs(buf, boundary=0, since_last_play=True, count=2) == ["c", "d"]


def test_get_logs_default_is_non_destructive(monkeypatch):
    monkeypatch.setattr(sim, "_log_buffer", ["x", "y"], raising=False)
    monkeypatch.setattr(sim, "_play_boundary", 0, raising=False)
    monkeypatch.setattr(sim, "_ensure_log_listener", lambda: None)
    result = sim.get_logs(adapter=None)  # defaults: clear=False
    assert result["logs"] == ["x", "y"]
    assert sim._log_buffer == ["x", "y"]  # buffer intact


def test_append_and_mark_boundary_scopes_new_run(monkeypatch):
    monkeypatch.setattr(sim, "_log_buffer", [], raising=False)
    monkeypatch.setattr(sim, "_play_boundary", 0, raising=False)
    monkeypatch.setattr(sim, "_ensure_log_listener", lambda: None)
    sim.append_log("[PRINT] old")
    sim.mark_play_boundary()
    sim.append_log("[PRINT] new")
    result = sim.get_logs(adapter=None, since_last_play=True)
    assert result["logs"] == ["[PRINT] new"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_log_buffer.py -v`
Expected: FAIL (`_select_logs`, `append_log`, `mark_play_boundary`, `_play_boundary` do not exist).

- [ ] **Step 3: Rewrite the log section of the handler**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py`, replace the log section (lines 150-193) with:

```python
# ── Log buffer for get_logs ───────────────────────────────────────────────────

_log_buffer: list = []
_log_listener_active: bool = False
_play_boundary: int = 0
_MAX_LOG_BUFFER = 500


def append_log(entry: str) -> None:
    """Append an entry to the shared log buffer, trimming to the cap."""
    _log_buffer.append(entry)
    if len(_log_buffer) > _MAX_LOG_BUFFER:
        # Keep the boundary consistent when we drop from the front.
        global _play_boundary
        _log_buffer.pop(0)
        if _play_boundary > 0:
            _play_boundary -= 1


def mark_play_boundary() -> None:
    """Record the buffer position at the current timeline Play."""
    global _play_boundary
    _play_boundary = len(_log_buffer)


def _select_logs(buffer: list, boundary: int, since_last_play: bool, count: int) -> list:
    """Pure selector: entries after the Play boundary (optional), capped to count."""
    scoped = buffer[boundary:] if since_last_play else buffer
    return scoped[-count:]


def _ensure_log_listener():
    """Register a carb log listener that captures warnings and errors."""
    global _log_listener_active
    if _log_listener_active:
        return

    import omni.log

    logger = omni.log.get_log()

    def _on_log(source, level, filename, function_name, module_name, line, message, pid, tid, timestamp):
        if level.value >= omni.log.Level.WARN.value:
            level_name = "WARN" if level == omni.log.Level.WARN else "ERROR"
            append_log(f"[{level_name}] [{source}] {message}")

    logger.set_channel_enabled("*", True, omni.log.SettingBehavior.OVERRIDE)
    logger.add_message_consumer(_on_log)
    _log_listener_active = True


def get_logs(
    adapter: IsaacAdapterBase, clear: bool = False, count: int = 100, since_last_play: bool = True
) -> Dict[str, Any]:
    """Return recent WARN/ERROR + [PRINT] log messages, scoped to the current run."""
    try:
        _ensure_log_listener()
        logs = _select_logs(_log_buffer, _play_boundary, since_last_play, count)
        if clear:
            _log_buffer.clear()
            mark_play_boundary()
        return {
            "status": "success",
            "log_count": len(logs),
            "logs": logs,
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}
```

Update the `get_logs` registry lambda (line 41) to forward the new param:

```python
    registry["simulation.get_logs"] = lambda **p: get_logs(adapter, **p)
```

(No change needed if it already forwards `**p`; keep it as `lambda **p: get_logs(adapter, **p)`.)

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_log_buffer.py -v`
Expected: PASS.

- [ ] **Step 5: Feed captured stdout into the buffer (print capture)**

In `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py`, in `execute_script` (success and error returns, lines 1053-1066) and `reload_script` (lines 1127-1140), after computing `captured_out.getvalue()`, route non-empty stdout into the buffer:

```python
            out = captured_out.getvalue()
            if out.strip():
                try:
                    from ..handlers.simulation import append_log

                    for line in out.splitlines():
                        append_log(f"[PRINT] {line}")
                except Exception:
                    pass
```

Insert this just before each `return {...}` that includes `"stdout": captured_out.getvalue()`, reusing `out` in the returned dict (`"stdout": out`). Apply the same change in `v5.py` `execute_script` / `reload_script`.

- [ ] **Step 6: Eager listener + Play boundary in on_startup**

Read `isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py` around lines 53-61 first. Then in `on_startup`, after `register_all_handlers(...)`, add eager registration and a Play-boundary subscription:

```python
        # Capture logs from extension load so early diagnostics are not missed,
        # and mark a run boundary on each timeline Play so get_isaac_logs can
        # scope to the current run.
        try:
            from .handlers.simulation import _ensure_log_listener, mark_play_boundary
            import omni.timeline

            _ensure_log_listener()
            self._play_sub = (
                omni.timeline.get_timeline_interface()
                .get_timeline_event_stream()
                .create_subscription_to_pop_by_type(
                    int(omni.timeline.TimelineEventType.PLAY),
                    lambda _e: mark_play_boundary(),
                )
            )
        except Exception as _e:
            print("log listener / play-boundary setup skipped:", _e)
```

(If `extension.py` already stores subscriptions on `self`, follow that naming. The `except` keeps a headless/stubbed environment working.)

- [ ] **Step 7: Update the tool params/docstring + test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_get_isaac_logs_has_since_last_play_and_nondestructive_default():
    import ast

    src = _read_tool_source("simulation.py")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "get_isaac_logs":
            defaults = {a.arg: d for a, d in zip(node.args.args[-len(node.args.defaults) :], node.args.defaults)}
            assert "since_last_play" in {a.arg for a in node.args.args}
            # clear defaults to False (non-destructive)
            clear_default = defaults.get("clear")
            assert isinstance(clear_default, ast.Constant) and clear_default.value is False
            return
    raise AssertionError("get_isaac_logs tool not found")
```

In `isaac_mcp/tools/simulation.py`, replace the `get_isaac_logs` tool (lines 126-143) with:

```python
    @mcp.tool("get_isaac_logs")
    def get_isaac_logs(clear: bool = False, count: int = 100, since_last_play: bool = True) -> str:
        """Diagnostic tool: recent WARN/ERROR logs plus captured print() output.

        Captures carb.log_*/omni.log WARN+ERROR and stdout from execute_script /
        reload_script (tagged [PRINT]). Plain print() outside those captured
        contexts may not appear.

        Defaults are agent-friendly: non-destructive (clear=False) and scoped to
        the current run (since_last_play=True) so you see logs from what you just
        did, not stale entries from previous runs.

        Args:
            clear: If True, empty the buffer after reading. Default False.
            count: Maximum number of log entries to return.
            since_last_play: If True (default), return only entries since the last
                timeline Play. Set False for the full buffer.
        """
        try:
            conn = get_connection()
            result = conn.send_command(
                "simulation.get_logs",
                {"clear": clear, "count": count, "since_last_play": since_last_play},
            )
            return json.dumps(result, indent=2)
        except Exception as e:
            return json.dumps({"status": "error", "message": str(e)})
```

- [ ] **Step 8: Run the full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add tests/test_log_buffer.py tests/test_tool_docstrings.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/handlers/simulation.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py \
        isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py \
        isaac_mcp/tools/simulation.py
git commit -m "feat: get_isaac_logs run-scoped, eager listener, print() capture (#4/#5)"
```

---

### Task 7: #6 + instruction gaps — `execute_script` caution & ScriptNode contract

**Files:**
- Modify: `isaac_mcp/tools/simulation.py:197-215` (`execute_script` docstring)
- Modify: `isaac_mcp/server.py:58-95` (`_INSTRUCTIONS`: add the missing contracts)
- Test: `tests/test_tool_docstrings.py` (extend)

**Interfaces:**
- Consumes: nothing.
- Produces: nothing.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_tool_docstrings.py`:

```python
def test_execute_script_warns_about_live_graph():
    src = _read_tool_source("simulation.py")
    assert "ScriptNode" in src
    assert "silently" in src.lower()
    assert "stop" in src.lower()


def test_server_instructions_cover_contracts():
    src = _read_server_source()
    assert "resets to spawn" in src.lower() or "spawn state" in src.lower()  # stop (#8)
    assert "[PRINT]" in src  # log capture (#5)
    assert "silently" in src.lower()  # execute_script (#6)
    assert "silent" in src.lower()  # ScriptNode write failures
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_tool_docstrings.py -v -k "live_graph or cover_contracts"`
Expected: FAIL.

- [ ] **Step 3: Update the execute_script docstring**

In `isaac_mcp/tools/simulation.py`, insert a caution paragraph into the `execute_script` docstring (after the USE line, before the "For persistent controllers" line, ~line 210):

```python
        CAUTION: touching an articulation controlled by a running ScriptNode /
        Action Graph can silently break its control path (no error is raised).
        While a graph is running, read-only diagnostics (get_prim_info,
        get_physics_state, get_joint_positions, get_isaac_logs) are safe, but
        stop_simulation before using execute_script or named write tools on the
        same articulation.
```

- [ ] **Step 4: Update the server instruction block**

In `isaac_mcp/server.py`, append a `### Contracts` section to `_INSTRUCTIONS` (before the closing `"""`, after Tool Priority, line 94):

```python
### Contracts (silent-failure map)
- step_simulation is authoritative and freezes the timeline; it errors if the
  timeline is already playing. Never play during the debug loop (see Debug Loop).
- stop_simulation resets the scene to spawn state (state at first Play).
- get_isaac_logs shows carb.log_*/omni.log WARN+ERROR plus captured stdout
  tagged [PRINT]; plain print() outside execute_script/reload_script may not
  appear. Defaults are non-destructive and scoped to the current run.
- execute_script can silently disturb a live Action Graph / ScriptNode that
  controls the same articulation — stop the graph first.
- ScriptNode physics contract: physics must be initialised before articulation
  writes take effect; such write failures are SILENT (not raised). Follow the
  WARMUP pattern (skip ~30 frames, then World.initialize_physics() +
  robot.initialize()).
```

- [ ] **Step 5: Run the full suite**

Run: `python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tests/test_tool_docstrings.py isaac_mcp/tools/simulation.py isaac_mcp/server.py
git commit -m "docs: execute_script live-graph caution + server-instruction contracts (#6, gaps)"
```

---

### Task 8: Changelog + final verification

**Files:**
- Modify: `CHANGELOG.md` (top of the unreleased/current section)
- Test: full suite

- [ ] **Step 1: Add a changelog entry**

At the top of `CHANGELOG.md`, add under an Unreleased/Fixed heading (match the file's existing style):

```markdown
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
```

- [ ] **Step 2: Run the full suite + linters**

Run: `python -m pytest tests/ -q && ruff check .`
Expected: PASS / no new lint errors.

- [ ] **Step 3: Commit**

```bash
git add CHANGELOG.md
git commit -m "docs: changelog for Isaac Sim MCP tool hardening"
```

---

## Self-Review

**Spec coverage:**
- #1 step determinism → Task 2 (handler guard + docstrings + instructions). ✔
- #2 inline action graph → Task 4. ✔
- #3 ScriptNode reload → Task 5. ✔
- #4/#5 logs (eager listener, since_last_play, non-destructive, print capture) → Task 6. ✔ (run-id/sequence + min_level intentionally omitted per spec trim.)
- #6 execute_script interference docs → Task 7. ✔
- #7 create_object scale docs → Task 1. ✔
- #8 stop resets → Task 3. ✔
- Server-instruction gaps → Task 7 (`### Contracts`). ✔
- Two-debug-mode distinction (spec #6) → Task 2 Debug Loop block + Task 7 contracts. ✔

**Placeholder scan:** No TBD/TODO; every code step shows the code. Live-only paths (og/World/SimulationManager) use AST presence tests + smoke_test additions, stated explicitly in Global Constraints.

**Type/name consistency:** `force_recompile_scriptnode(graph, node)` defined in Task 5 (handlers/graphs.py) and referenced by v5/v6 `_recompile_scriptnodes_for_file` in the same task. `append_log`/`mark_play_boundary`/`_select_logs`/`_play_boundary` defined in Task 6 and used by the adapters (print capture) and `extension.py` (boundary) in the same task. `get_simulation_state()["timeline_state"]` consumed by Task 2 is produced by the existing adapters. No forward references across tasks without a definition.

**Refinement vs spec:** Spec put the #1 guard "in v5/v6 step"; the plan places it once in the handler using the existing `get_simulation_state()` — same fail-loud behaviour, DRY, and unit-testable without a live timeline. Consistent with spec intent.
