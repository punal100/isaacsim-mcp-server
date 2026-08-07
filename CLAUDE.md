# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
uv sync                              # install deps (creates .venv)
uv run pre-commit install            # ruff lint+format on commit
uv run pytest                        # full unit suite (no Isaac Sim needed)
uv run pytest tests/test_adapter_v6.py::test_get_adapter_returns_v6_when_version_6   # single test
uv run ruff check . && uv run ruff format .                                 # what CI enforces
uv run python add_license_headers.py # prepend the MIT header to new .py files
```

Live testing (unit tests alone are **not** sufficient — CONTRIBUTING requires verifying in a running simulator, and stating the Isaac Sim version in the PR):

```bash
./scripts/run_isaac_sim.sh              # Kit + extension; --newton / --physx / ISAACSIM_ENGINE
./scripts/dev_mcp_server.sh             # MCP server + extension hot-reload watcher
python scripts/smoke_test_v6.py         # one socket command per V6 surface, pass/fail per check
./.venv/bin/python -m mcp dev ./isaac_mcp/server.py   # MCP inspector on :5173
```

`dev_mcp_server.sh` reloads handler/adapter modules inside the running Kit process, so extension edits do not need an Isaac Sim restart. MCP-server-side edits (`isaac_mcp/`) do need the MCP client to restart the server.

## Architecture

Two processes, one JSON-over-TCP hop (localhost:8766, `ISAAC_MCP_PORT` to change):

```
MCP client → isaac_mcp/ (FastMCP, stdio) → socket → isaac.sim.mcp_extension/ (Omniverse ext) → adapter → Isaac Sim APIs
```

**MCP side** — [isaac_mcp/server.py](isaac_mcp/server.py) builds `FastMCP`, attaches the `_INSTRUCTIONS` block (the agent-facing contract doc: workflow, debug loop, silent-failure map), and calls `register_all_tools`. Each module in [isaac_mcp/tools/](isaac_mcp/tools/) exposes `register_tools(mcp, get_connection)`; every tool serializes params, calls `conn.send_command("<category>.<verb>", params)`, and returns `json.dumps(result)`. Tools never raise — errors come back as `{"status": "error", "message": ...}` JSON strings.

[isaac_mcp/connection.py](isaac_mcp/connection.py) holds a singleton socket. It probes with `MSG_PEEK` before each send because the MCP server outlives Kit restarts; a stale socket is redialled rather than retried after failure (replaying a `create_robot`/`delete` would be worse than the error).

**Extension side** — [extension.py](isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py) picks an adapter, fills a `Dict[str, handler]` registry, and starts [socket_server.py](isaac.sim.mcp_extension/isaac_sim_mcp_extension/socket_server.py). Command strings are the wire contract (`"lighting.create"`, `"simulation.step"`, …) and must match on both sides. A handler returning anything other than `{"status": "success", ...}` is converted into an error response by `_execute_command`.

**Adapters** — [adapters/base.py](isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/base.py) is the ABC plus the shared physics helpers; [v5.py](isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v5.py) targets Isaac Sim 5.1 (`isaacsim.core.api/.prims/.utils`), [v6.py](isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py) targets 6.0 PhysX **and** Newton (`isaacsim.core.experimental.*`, `SimulationManager`, `isaacsim.sensors.experimental.rtx`). `get_adapter()` reads `isaacsim.core.version.get_version()` — a string on 5.x, a tuple on 6.0 — and falls back to V5 on detection failure. Handlers must stay version-agnostic: anything version-specific belongs behind an abstract method.

### Adding a tool

Five places, in order: `isaac_mcp/tools/<category>.py` → new command string → `handlers/<category>.py` `register()` → abstract method in `adapters/base.py` implemented in **both** `v5.py` and `v6.py` → if you add a new *module*, add it to the reload lists in `scripts/dev_mcp_server.sh` and to both `__init__.py` module lists. `extension.toml` dependencies gate what the extension can import on each version — V6-only extensions must be `{ optional = true }` or 5.1 fails to load.

## Conventions and traps

- Every `.py` file carries the MIT header block; `add_license_headers.py` applies it.
- Handlers **must not** call `omni.kit.app.update()` — they run as an asyncio task on Kit's main loop and pumping it crashes Kit (see the comment in `socket_server._dispatch_command`; `v6.step` uses `SimulationManager.step` instead).
- Several tests assert on *source substrings* of tool docstrings and the server instruction block ([tests/test_tool_docstrings.py](tests/test_tool_docstrings.py)) — rewording docs breaks tests, deliberately. Update both together.
- [tests/conftest.py](tests/conftest.py) stubs `carb`, `omni`, `pxr`, and `numpy` into `sys.modules` so the extension imports outside Kit. New runtime imports at module scope may need a stub added there; `pytest.ini_options.pythonpath` points at `isaac.sim.mcp_extension`.
- Behavioral contracts encoded in `base.py` and worth preserving — each fixed a silent-wrong-answer bug: exactly one `PhysicsScene` on the stage (a second one zeroes every velocity read), gravity written as USD direction+magnitude, Action Graphs suspended during `step` (else a ScriptNode overwrites stepped joint targets), `_ensure_physics_world` no-ops until a PhysicsScene exists.
- Debug loop is **step-only on a frozen timeline**; `play` is for the final Action-Graph run. The two modes do not mix — that split is repeated in the server instructions, tool docstrings, and adapter comments.
- ScriptNode scripts must define `setup(db)`/`compute(db)`; legacy mode (no `compute`) breaks exec scoping. Full rules in [isaac.sim.mcp_extension/.cursorrules](isaac.sim.mcp_extension/.cursorrules); working example in [demo/franka_pick_place.py](demo/franka_pick_place.py).
- Design docs and plans for past feature work live in [docs/superpowers/](docs/superpowers/); `CHANGELOG.md` records why each adapter API was chosen.
