# MIT License
#
# Copyright (c) 2023-2025 omni-mcp
# Copyright (c) 2026 whats2000
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""V5 get_physics_state must read velocity from a source that actually has it."""

import ast
import os

V5 = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
    "v5.py",
)


def _func_src(name):
    with open(V5) as f:
        text = f.read()
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(text, node)
    raise AssertionError(f"{name} not found in v5.py")


def test_velocity_not_read_from_rigidbody_transformation():
    """get_rigidbody_transformation() returns only {position, rotation, ret_val}.

    Reading velocity from it silently yields [0, 0, 0] for every body, so a
    falling object looks identical to one at rest. Verified live: a cube moving
    at 15 m/s reported zero velocity through that call.
    """
    code = "\n".join(line.split("#", 1)[0] for line in _func_src("get_physics_state").splitlines())
    assert "get_rigidbody_transformation" not in code, (
        "get_rigidbody_transformation has no velocity keys — velocity would always read zero"
    )


def test_velocity_read_from_physics_attributes():
    code = _func_src("get_physics_state")
    assert "physics:velocity" in code
    assert "physics:angularVelocity" in code


def test_angular_velocity_converted_to_radians():
    """USD stores angularVelocity in deg/s; this API reports radians elsewhere."""
    code = _func_src("get_physics_state")
    assert "radians" in code, "angular velocity must be converted from USD deg/s to rad/s"


def test_v6_does_not_use_usd_velocity_attributes():
    """V6 must keep reading the physics-tensors view, not USD attributes.

    physics:velocity is only written back by PhysX. Isaac Sim 6.0 can run the
    Newton backend, which keeps state in its own buffers, so the USD attribute
    may be stale or zero there. The tensor view is backend-neutral.
    """
    import ast
    import os

    v6_path = os.path.join(
        os.path.dirname(__file__), "..", "isaac.sim.mcp_extension", "isaac_sim_mcp_extension", "adapters", "v6.py"
    )
    with open(v6_path) as f:
        text = f.read()
    tree = ast.parse(text)
    src = ""
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "get_physics_state":
            src = ast.get_source_segment(text, node)
    assert src, "v6 get_physics_state not found"
    assert "physics:velocity" not in src, "V6/Newton must not read PhysX USD write-back attributes"
    assert "get_velocities" in src, "V6 should read velocities from the physics tensors view"


def test_v5_warns_when_usd_writeback_disabled():
    """A disabled write-back setting must be reported, not silently read as zero."""
    code = _func_src("get_physics_state")
    assert "updateVelocitiesToUsd" in code
    assert "velocity_warning" in code


def _v6_adapter_with_view(monkeypatch, view):
    """A V6 adapter whose physics simulation view is `view` (None = unavailable)."""
    import importlib
    import sys
    import types

    monkeypatch.setitem(
        sys.modules,
        "isaacsim.core.simulation_manager",
        types.SimpleNamespace(
            SimulationManager=type(
                "SM",
                (),
                {
                    "get_active_physics_engine": classmethod(lambda cls: "physx"),
                    "get_physics_simulation_view": classmethod(lambda cls: view),
                },
            )
        ),
    )
    monkeypatch.setitem(sys.modules, "isaacsim.core.version", types.SimpleNamespace(get_version=lambda: "6.0.0"))

    import isaac_sim_mcp_extension.adapters.v6 as v6_mod

    importlib.reload(v6_mod)

    from pxr import UsdPhysics

    class _RigidBodyAPI:
        def __init__(self, prim):
            pass

        def GetKinematicEnabledAttr(self):
            return types.SimpleNamespace(Get=lambda: False)

    monkeypatch.setattr(UsdPhysics, "RigidBodyAPI", _RigidBodyAPI, raising=False)
    monkeypatch.setattr(UsdPhysics, "MassAPI", type("MassAPI", (), {}), raising=False)
    monkeypatch.setattr(UsdPhysics, "CollisionAPI", type("CollisionAPI", (), {}), raising=False)

    class _Prim:
        def IsValid(self):
            return True

        def HasAPI(self, schema):
            return schema is UsdPhysics.RigidBodyAPI

    adapter = v6_mod.IsaacAdapterV6()
    monkeypatch.setattr(adapter, "get_stage", lambda: types.SimpleNamespace(GetPrimAtPath=lambda p: _Prim()))
    return adapter


def test_v6_says_when_velocity_could_not_be_measured(monkeypatch):
    """Zeros from an unavailable view must not be reported as a body at rest.

    V5 sets velocity_warning when PhysX write-back is off rather than reporting
    a moving body as stationary. V6 pre-seeded [0,0,0] and swallowed every
    failure, so "no physics view", "view invalidated" and "genuinely at rest"
    were indistinguishable -- the same silent wrong answer, without the warning.
    """
    adapter = _v6_adapter_with_view(monkeypatch, None)

    result = adapter.get_physics_state("/World/Cube")

    assert result["linear_velocity"] == [0.0, 0.0, 0.0]
    assert "velocity_warning" in result, "an unmeasured velocity was reported as a measurement"


def test_v6_does_not_warn_when_the_view_answered(monkeypatch):
    """The warning must be scoped to failures, not blanket every read."""

    class _Array:
        """Enough of an ndarray for the read: .size and .reshape(-1)."""

        def __init__(self, flat):
            self._flat = flat

        @property
        def size(self):
            return len(self._flat)

        def reshape(self, _shape):
            return self._flat

    class _Velocities:
        def numpy(self):
            return _Array([1.0, 2.0, 3.0, 0.4, 0.5, 0.6])

    class _RbView:
        def get_velocities(self):
            return _Velocities()

    class _View:
        def create_rigid_body_view(self, paths):
            return _RbView()

    adapter = _v6_adapter_with_view(monkeypatch, _View())

    result = adapter.get_physics_state("/World/Cube")

    assert result["linear_velocity"] == [1.0, 2.0, 3.0]
    assert "velocity_warning" not in result, "a successful measurement must not carry a warning"
