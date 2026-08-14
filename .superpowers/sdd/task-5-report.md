# Task 5 Report: V6 Articulation Wrapper

## Status: DONE

## What was done

Replaced the five `NotImplementedError` stubs in `adapters/v6.py` (Robots section) with full implementations:

- `create_xform_prim(path)` → `XformPrim(paths=[path])`
- `create_articulation(path, name)` → `Articulation(paths=[path])`
- `_new_articulation(path)` → `Articulation(paths=[path])` (private helper, fresh per call)
- `get_robot_joint_info(path)` — tries Articulation API first, falls back to USD traversal
- `set_joint_positions(path, positions, joint_indices)` — warp path + USD-drive fallback
- `_set_joint_drive_targets(path, positions, joint_indices)` — verbatim V5 fallback (pure pxr)
- `_get_joint_names(path)` — tries Articulation.dof_names, falls back to USD traversal
- `get_joint_positions(path)` — warp path + USD-drive fallback
- `get_joint_config(path)` — rich joint info with stiffness/damping warnings

Also appended `test_v6_set_joint_positions_calls_set_dof_position_targets` to `tests/test_adapter_v6.py`.

## Deviation from Brief

The brief's `set_joint_positions` used `wp.array(np.asarray([list(positions)], dtype=np.float32), dtype=wp.float32)`. The test environment has a stub numpy (no `asarray`), so this caused a silent `AttributeError` that was swallowed by `except Exception: pass`, falling through to the USD fallback and ultimately failing.

**Fix:** Changed to `wp.array([list(positions)], dtype=wp.float32)` — passes a plain Python nested list directly to the warp array factory. This satisfies the test assertion `list(captured["positions"][0]) == [0.1, 0.2, 0.3]` identically, and in production warp accepts both numpy arrays and Python lists.

Similarly `idx_arr` uses `list(joint_indices)` instead of `np.asarray(joint_indices, dtype=np.int32)`.

## TDD Evidence

**RED:** `pytest tests/test_adapter_v6.py::test_v6_set_joint_positions_calls_set_dof_position_targets` → FAILED with `NotImplementedError: set_joint_positions: not yet implemented for V6`. Expected because the stub raised NotImplementedError.

**GREEN:** After implementation → PASSED.

**Full suite:** `pytest tests/test_adapter_v6.py` → 8 passed.

## Files Modified

- `isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py`
- `tests/test_adapter_v6.py`

---

## Review Fix: Restore np.asarray per Brief Spec

### What Changed

**`isaac.sim.mcp_extension/isaac_sim_mcp_extension/adapters/v6.py` — lines 391 and 393:**

Restored the `np.asarray` calls in `set_joint_positions` per the Task 5 brief:

- Line 391 (before): `wp.array([list(positions)], dtype=wp.float32)`
- Line 391 (after): `wp.array(np.asarray([list(positions)], dtype=np.float32), dtype=wp.float32)`

- Line 393 (before): `wp.array(list(joint_indices), dtype=wp.int32)`
- Line 393 (after): `wp.array(np.asarray(joint_indices, dtype=np.int32), dtype=wp.int32)`

**`tests/conftest.py` — lines 115-121:**

Added `asarray`, `float32`, and `int32` attributes to the numpy stub in `_install_isaac_stubs()` so the production code path is exercisable without hitting `AttributeError`. The root cause of the original deviation was that the stub numpy (installed by conftest when real numpy is absent) lacked `asarray` and dtype sentinels (`float32`, `int32`). The `asarray` stub is a passthrough (`lambda *args, **kwargs: args[0]`); the dtype sentinels are plain string sentinels (the stub `asarray` ignores the `dtype` kwarg, so their value is irrelevant).

### Test Command and Output

```
python -m pytest tests/test_adapter_v6.py -v
```

```
============================= test session starts ==============================
collected 8 items

tests/test_adapter_v6.py::test_get_adapter_returns_v5_when_version_5 PASSED [ 12%]
tests/test_adapter_v6.py::test_get_adapter_returns_v6_when_version_6 PASSED [ 25%]
tests/test_adapter_v6.py::test_get_adapter_falls_back_to_v5_when_detection_fails PASSED [ 37%]
tests/test_adapter_v6.py::test_detect_version_reads_isaacsim_core_version PASSED [ 50%]
tests/test_adapter_v6.py::test_detect_version_returns_zero_on_failure PASSED [ 62%]
tests/test_adapter_v6.py::test_v6_create_prim_calls_experimental_define_prim PASSED [ 75%]
tests/test_adapter_v6.py::test_v6_ensure_physics_world_calls_simulation_manager PASSED [ 87%]
tests/test_adapter_v6.py::test_v6_set_joint_positions_calls_set_dof_position_targets PASSED [100%]

============================== 8 passed in 0.03s ==============================
```

### Compliance Statement

The production code in `set_joint_positions` now exactly matches the Task 5 brief's spec: both `wp.array` calls wrap their inputs with `np.asarray` (with explicit shape/dtype) before passing to warp, giving unambiguous shape semantics across warp versions.
