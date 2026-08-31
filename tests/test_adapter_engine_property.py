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

# SPDX-License-Identifier: MIT
"""Handlers must ask the adapter what engine it is, not sniff a private attribute.

CLAUDE.md: handlers stay version-agnostic and version-specific behaviour lives
behind an adapter method. Four handlers reached for `adapter._engine` instead --
a V6-only property -- with two different fallbacks for the V5 case
(`getattr(adapter, "_engine", "physx")` and `getattr(adapter, "_engine", None)`),
so the same question had two answers depending on which handler asked.
"""

import ast
import os

import pytest
from isaac_sim_mcp_extension.adapters.base import IsaacAdapterBase

HANDLERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "handlers",
)


class _Adapter(IsaacAdapterBase):
    """Exposes only the public surface a handler is allowed to use."""

    def __init__(self, engine="physx", strands=False):
        self._e = engine
        self._strands = strands
        self.primed = False

    @property
    def engine(self):
        return self._e

    def strands_first_rtx_camera(self):
        return self._strands

    def _ensure_physics_world(self):
        self.primed = True

    def create_physics_scene(self, gravity=None, scene_name="PhysicsScene"):
        return "/PhysicsScene"

    def get_stage(self):
        return None


_Adapter.__abstractmethods__ = frozenset()


def test_base_adapter_reports_physx():
    """V5 is PhysX-only, so the base answer is a real answer, not a fallback."""
    assert _Adapter().engine == "physx"
    assert IsaacAdapterBase.ENGINE_PHYSX == "physx"
    assert IsaacAdapterBase.ENGINE_NEWTON == "newton"


@pytest.mark.parametrize("handler_file", ["scene.py", "robots.py", "sensors.py"])
def test_no_handler_sniffs_the_private_engine_attribute(handler_file):
    with open(os.path.join(HANDLERS, handler_file)) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        # Both spellings: adapter._engine, and the getattr(adapter, "_engine")
        # form the handlers actually use -- a string literal, which an
        # Attribute-only check walks straight past.
        if isinstance(node, ast.Attribute) and node.attr == "_engine":
            raise AssertionError(f"{handler_file} reads adapter._engine; use the public `engine` property")
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "getattr":
            if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant) and node.args[1].value == "_engine":
                raise AssertionError(
                    f'{handler_file} reads getattr(adapter, "_engine"); use the public `engine` property'
                )


def test_newton_drive_warning_uses_the_public_property():
    from isaac_sim_mcp_extension.handlers.robots import engine_drive_warning

    assert engine_drive_warning(_Adapter(engine="newton")) is not None
    assert engine_drive_warning(_Adapter(engine="physx")) is None


def test_first_camera_warning_is_the_adapters_call():
    """V5 removes every camera, so the warning would be false there."""
    from isaac_sim_mcp_extension.handlers.sensors import _first_rtx_camera

    assert _first_rtx_camera(_Adapter(strands=False), "/World/C") is False

    strands = _Adapter(strands=True)
    assert _first_rtx_camera(strands, "/World/C") is True
    assert _first_rtx_camera(strands, "/World/C2") is False, "the warning must fire once per session"


def test_handler_does_not_write_adapter_state():
    """A handler reaching into the adapter to stash a flag is the rule inverted."""
    with open(os.path.join(HANDLERS, "sensors.py")) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Attribute) and tgt.attr == "_first_rtx_camera_path":
                    raise AssertionError("sensors.py assigns adapter._first_rtx_camera_path; ask the adapter instead")
