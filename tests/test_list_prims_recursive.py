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

"""list_prims walks one level by default and the whole subtree on request.

It documented "all prims in the scene" while returning only immediate children,
so a camera nested under a robot was invisible to a /World listing that reported
success. That is issue #30, and it misled the 0.6.0 release sweep: /World
present after a clear_scene read as an empty scene while a lidar was still under
it.
"""

import ast
import os

from isaac_sim_mcp_extension.adapters.base import collect_prims

ADAPTERS = os.path.join(
    os.path.dirname(__file__), "..", "isaac.sim.mcp_extension", "isaac_sim_mcp_extension", "adapters"
)


class _Prim:
    """Minimal stand-in for a Usd.Prim: a path, a type name, and children."""

    def __init__(self, path, type_name="", children=()):
        self._path = path
        self._type = type_name
        self._children = list(children)

    def GetPath(self):
        return self._path

    def GetTypeName(self):
        return self._type

    def GetAllChildren(self):
        return self._children


def _world():
    """A stage shaped like the one that exposed this: a nested sensor."""
    return _Prim(
        "/World",
        "",
        [
            _Prim("/World/PhysicsScene", "PhysicsScene"),
            _Prim(
                "/World/Arm",
                "Xform",
                [
                    _Prim("/World/Arm/EyeCam", "Camera"),
                    _Prim("/World/Arm/base", "Xform", [_Prim("/World/Arm/base/mesh", "Mesh")]),
                ],
            ),
            _Prim("/World/Camera", "Camera"),
        ],
    )


def test_default_stays_shallow():
    """Existing callers must keep the output they have — a robot subtree would
    otherwise turn a three-row answer into hundreds, which is exactly the
    context blow-up this tool's summary style avoids."""
    rows = collect_prims(_world())

    assert [r["path"] for r in rows] == ["/World/PhysicsScene", "/World/Arm", "/World/Camera"]


def test_recursive_reaches_nested_prims():
    rows = collect_prims(_world(), recursive=True)
    paths = [r["path"] for r in rows]

    assert "/World/Arm/EyeCam" in paths
    assert "/World/Arm/base/mesh" in paths, "recursion must go deeper than one level"
    assert len(paths) == 6


def test_type_filter_still_descends_through_non_matching_prims():
    """The case that makes this a correctness bug rather than a convenience gap.

    /World/Arm is an Xform, so a Camera filter skips it — but the camera it
    contains must still be found, or `list_prims(prim_type="Camera")` reports
    success while missing the very prim the caller is looking for.
    """
    rows = collect_prims(_world(), prim_type="Camera", recursive=True)

    assert sorted(r["path"] for r in rows) == ["/World/Arm/EyeCam", "/World/Camera"]


def test_shallow_type_filter_is_unchanged():
    rows = collect_prims(_world(), prim_type="Camera")

    assert [r["path"] for r in rows] == ["/World/Camera"]


def test_both_adapters_use_the_shared_walker():
    """v5 and v6 carried the identical loop; neither should keep its own copy."""
    for name in ("v5.py", "v6.py"):
        with open(os.path.join(ADAPTERS, name)) as f:
            tree = ast.parse(f.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "list_prims":
                called = {n.func.id for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
                assert "collect_prims" in called, f"{name} still walks prims itself"
                args = {a.arg for a in node.args.args} | {a.arg for a in node.args.kwonlyargs}
                assert "recursive" in args, f"{name}.list_prims does not accept recursive"
                break
        else:
            raise AssertionError(f"list_prims not found in {name}")
