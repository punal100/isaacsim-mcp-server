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

"""Tests for fixes verified live against Isaac Sim (Copilot review findings)."""

import os
from unittest.mock import MagicMock

ADAPTERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
)


# ── #1: create_object normalizes non-canonical type casing ────────────────────
# Verified live: object_type="cube" produced a typeless prim (type "cube", no
# actual_size); "Cube" produced a real UsdGeom.Cube. The handler must normalize.


def test_create_object_normalizes_lowercase_type():
    from isaac_sim_mcp_extension.handlers import objects

    adapter = MagicMock()
    objects.create(adapter, object_type="cube", prim_path="/World/x")

    # create_prim is called first, before any later step can fail — assert it
    # received the canonical "Cube", not "cube".
    _args, kwargs = adapter.create_prim.call_args
    assert kwargs.get("prim_type") == "Cube"


def test_create_object_leaves_canonical_and_unknown_types_untouched():
    from isaac_sim_mcp_extension.handlers import objects

    for given, expected in (("Cube", "Cube"), ("SPHERE", "Sphere"), ("Xform", "Xform")):
        adapter = MagicMock()
        objects.create(adapter, object_type=given, prim_path="/World/x")
        _args, kwargs = adapter.create_prim.call_args
        assert kwargs.get("prim_type") == expected


# ── #2: get_simulation_state detects the PhysicsScene with IsA, not HasAPI ─────
# Verified live: HasAPI(UsdPhysics.Scene) returned False on a PhysicsScene prim
# (physics_dt stuck at 1/60); IsA(UsdPhysics.Scene) returned True (correct dt).


def test_get_simulation_state_uses_isa_for_physics_scene():
    for fname in ("v5.py", "v6.py"):
        with open(os.path.join(ADAPTERS, fname)) as f:
            src = f.read()
        assert "IsA(UsdPhysics.Scene)" in src, f"{fname}: physics-scene check must use IsA"
        assert "HasAPI(UsdPhysics.Scene)" not in src, f"{fname}: HasAPI never matches a typed schema"
