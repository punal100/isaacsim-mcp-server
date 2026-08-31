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
"""set_joint_positions must say whether the articulation took the command.

Reads gained provenance in this cycle; writes did not. When the articulation
command fails twice the adapters silently fall back to authoring USD drive
targets, and the handler reported a flat "Set joint positions on ..." for both.
A drive target written while the physics view is dead moves nothing until the
next initialize, so the two outcomes are not interchangeable.
"""

import sys
import types

import pytest
from isaac_sim_mcp_extension.adapters.base import IsaacAdapterBase
from isaac_sim_mcp_extension.handlers.robots import set_joints


class _Adapter(IsaacAdapterBase):
    """Enough adapter to drive the handler; command source is settable."""

    def __init__(self, source):
        self._source = source

    def __getattr__(self, name):
        raise AttributeError(name)

    def get_joint_positions(self, prim_path):
        return [0.0, 0.0, 0.0]

    def set_joint_positions(self, prim_path, positions, joint_indices=None):
        self._note_joint_command_source(self._source)

    def get_stage(self):
        class _Prim:
            def IsValid(self):
                return True

        return type("S", (), {"GetPrimAtPath": staticmethod(lambda p: _Prim())})()


_Adapter.__abstractmethods__ = frozenset()


def test_a_drive_target_write_is_flagged():
    adapter = _Adapter(IsaacAdapterBase.JOINT_COMMAND_DRIVE_TARGETS)

    out = set_joints(adapter, prim_path="/World/R", joint_positions=[0.1, 0.2, 0.3])

    assert out["status"] == "success"
    assert out["command_source"] == "drive_targets"
    assert "warning" in out, "a write the articulation never took was reported as if it had"


def test_an_articulation_write_is_not_flagged():
    adapter = _Adapter(IsaacAdapterBase.JOINT_COMMAND_ARTICULATION)

    out = set_joints(adapter, prim_path="/World/R", joint_positions=[0.1, 0.2, 0.3])

    assert out["status"] == "success"
    assert out["command_source"] == "articulation"
    assert "warning" not in out, "a normal write must not carry a warning"


@pytest.fixture(autouse=True)
def _isaac_modules(monkeypatch):
    """The module-scope imports set_joint_positions makes before any branch."""
    prims = types.ModuleType("isaacsim.core.prims")
    prims.SingleArticulation = object
    types_mod = types.ModuleType("isaacsim.core.utils.types")
    types_mod.ArticulationAction = object
    warp = types.ModuleType("warp")
    warp.array = lambda data, dtype=None: list(data)
    warp.float32 = "float32"
    exp_prims = types.ModuleType("isaacsim.core.experimental.prims")
    exp_prims.Articulation = object

    for name, mod in (
        ("isaacsim", types.ModuleType("isaacsim")),
        ("isaacsim.core", types.ModuleType("isaacsim.core")),
        ("isaacsim.core.prims", prims),
        ("isaacsim.core.utils", types.ModuleType("isaacsim.core.utils")),
        ("isaacsim.core.utils.types", types_mod),
        ("isaacsim.core.experimental", types.ModuleType("isaacsim.core.experimental")),
        ("isaacsim.core.experimental.prims", exp_prims),
        ("warp", warp),
    ):
        monkeypatch.setitem(sys.modules, name, mod)


def _stub_adapter(cls, applied, drive_calls):
    """A v5/v6 adapter whose articulation attempt succeeds or fails on demand."""
    adapter = cls.__new__(cls)
    adapter._try_articulation = lambda fn: (None, applied)
    adapter._set_joint_drive_targets = lambda *a, **k: drive_calls.append(a)
    return adapter


def test_v5_notes_the_fallback_when_the_articulation_refuses():
    """The handler's warning is dead unless the adapter records the fallback."""
    from isaac_sim_mcp_extension.adapters.v5 import IsaacAdapterV5

    calls = []
    adapter = _stub_adapter(IsaacAdapterV5, applied=False, drive_calls=calls)

    adapter.set_joint_positions("/World/R", [0.1, 0.2])

    assert calls, "the drive-target fallback did not run"
    assert adapter.joint_command_source == IsaacAdapterV5.JOINT_COMMAND_DRIVE_TARGETS


def test_v5_notes_the_articulation_when_it_takes_the_command():
    from isaac_sim_mcp_extension.adapters.v5 import IsaacAdapterV5

    calls = []
    adapter = _stub_adapter(IsaacAdapterV5, applied=True, drive_calls=calls)

    adapter.set_joint_positions("/World/R", [0.1, 0.2])

    assert not calls, "the fallback ran even though the articulation took the command"
    assert adapter.joint_command_source == IsaacAdapterV5.JOINT_COMMAND_ARTICULATION


def test_v6_notes_the_fallback_when_the_articulation_refuses():
    from isaac_sim_mcp_extension.adapters.v6 import IsaacAdapterV6

    calls = []
    adapter = _stub_adapter(IsaacAdapterV6, applied=False, drive_calls=calls)

    adapter.set_joint_positions("/World/R", [0.1, 0.2])

    assert calls, "the drive-target fallback did not run"
    assert adapter.joint_command_source == IsaacAdapterV6.JOINT_COMMAND_DRIVE_TARGETS


def test_v6_notes_the_articulation_when_it_takes_the_command():
    from isaac_sim_mcp_extension.adapters.v6 import IsaacAdapterV6

    calls = []
    adapter = _stub_adapter(IsaacAdapterV6, applied=True, drive_calls=calls)

    adapter.set_joint_positions("/World/R", [0.1, 0.2])

    assert not calls
    assert adapter.joint_command_source == IsaacAdapterV6.JOINT_COMMAND_ARTICULATION
