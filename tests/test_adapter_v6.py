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

"""Tests for the V6 adapter and version-aware dispatcher."""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture
def fake_v5(monkeypatch):
    mod = types.ModuleType("isaac_sim_mcp_extension.adapters.v5")
    class _V5:
        pass
    mod.IsaacAdapterV5 = _V5
    monkeypatch.setitem(sys.modules, "isaac_sim_mcp_extension.adapters.v5", mod)
    return _V5


@pytest.fixture
def fake_v6(monkeypatch):
    mod = types.ModuleType("isaac_sim_mcp_extension.adapters.v6")
    class _V6:
        pass
    mod.IsaacAdapterV6 = _V6
    monkeypatch.setitem(sys.modules, "isaac_sim_mcp_extension.adapters.v6", mod)
    return _V6


def test_get_adapter_returns_v5_when_version_5(fake_v5, fake_v6):
    from isaac_sim_mcp_extension.adapters import get_adapter
    with patch("isaac_sim_mcp_extension.adapters._detect_isaacsim_major_version", return_value=5):
        adapter = get_adapter()
    assert isinstance(adapter, fake_v5)


def test_get_adapter_returns_v6_when_version_6(fake_v5, fake_v6):
    from isaac_sim_mcp_extension.adapters import get_adapter
    with patch("isaac_sim_mcp_extension.adapters._detect_isaacsim_major_version", return_value=6):
        adapter = get_adapter()
    assert isinstance(adapter, fake_v6)


def test_get_adapter_falls_back_to_v5_when_detection_fails(fake_v5, fake_v6):
    from isaac_sim_mcp_extension.adapters import get_adapter
    with patch("isaac_sim_mcp_extension.adapters._detect_isaacsim_major_version", return_value=0):
        adapter = get_adapter()
    assert isinstance(adapter, fake_v5)


def test_detect_version_reads_isaacsim_core_version(monkeypatch):
    """When isaacsim.core.version.get_version returns a 6.x string, detection returns 6."""
    fake_version_mod = types.ModuleType("isaacsim.core.version")
    fake_version_mod.get_version = lambda: "6.0.0-rc.59+release.41464.5f2772bc.gl"
    fake_core_mod = types.ModuleType("isaacsim.core")
    fake_isaac_mod = types.ModuleType("isaacsim")
    monkeypatch.setitem(sys.modules, "isaacsim", fake_isaac_mod)
    monkeypatch.setitem(sys.modules, "isaacsim.core", fake_core_mod)
    monkeypatch.setitem(sys.modules, "isaacsim.core.version", fake_version_mod)

    from isaac_sim_mcp_extension.adapters import _detect_isaacsim_major_version
    # Module already loaded in earlier tests; force a fresh import path
    import importlib, isaac_sim_mcp_extension.adapters as adapters_mod
    importlib.reload(adapters_mod)
    assert adapters_mod._detect_isaacsim_major_version() == 6


def test_detect_version_returns_zero_on_failure(monkeypatch):
    """When neither isaacsim.core.version nor a VERSION file is reachable, returns 0."""
    for name in list(sys.modules):
        if name.startswith("isaacsim"):
            monkeypatch.delitem(sys.modules, name, raising=False)

    import importlib, isaac_sim_mcp_extension.adapters as adapters_mod
    importlib.reload(adapters_mod)
    assert adapters_mod._detect_isaacsim_major_version() == 0


def test_v6_create_prim_calls_experimental_define_prim(monkeypatch):
    """V6.create_prim must call isaacsim.core.experimental.utils.stage.define_prim."""
    define_prim_mock = MagicMock(return_value="prim-handle")
    fake_stage_mod = types.ModuleType("isaacsim.core.experimental.utils.stage")
    fake_stage_mod.define_prim = define_prim_mock
    fake_stage_mod.add_reference_to_stage = MagicMock()
    fake_stage_mod.delete_prim = MagicMock()
    for name in ("isaacsim", "isaacsim.core", "isaacsim.core.experimental", "isaacsim.core.experimental.utils"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "isaacsim.core.experimental.utils.stage", fake_stage_mod)

    # SimulationManager mock for __init__
    fake_sm_mod = types.ModuleType("isaacsim.core.simulation_manager")
    class _SM:
        @classmethod
        def get_active_physics_engine(cls):
            return "physx"
    fake_sm_mod.SimulationManager = _SM
    monkeypatch.setitem(sys.modules, "isaacsim.core.simulation_manager", fake_sm_mod)

    fake_version_mod = types.ModuleType("isaacsim.core.version")
    fake_version_mod.get_version = lambda: "6.0.0"
    monkeypatch.setitem(sys.modules, "isaacsim.core.version", fake_version_mod)

    import importlib
    import isaac_sim_mcp_extension.adapters.v6 as v6_mod
    importlib.reload(v6_mod)
    adapter = v6_mod.IsaacAdapterV6()
    result = adapter.create_prim("/World/Foo", "Xform")
    define_prim_mock.assert_called_once_with("/World/Foo", type_name="Xform")
    assert result == "prim-handle"


def test_v6_ensure_physics_world_calls_simulation_manager(monkeypatch):
    """V6._ensure_physics_world must use SimulationManager, not World."""
    sm_calls = []

    class _SM:
        @classmethod
        def get_active_physics_engine(cls):
            return "newton"

        @classmethod
        def setup_simulation(cls, dt=None, device=None):
            sm_calls.append(("setup_simulation", dt))

        @classmethod
        def initialize_physics(cls):
            sm_calls.append(("initialize_physics",))

    fake_sm_mod = types.ModuleType("isaacsim.core.simulation_manager")
    fake_sm_mod.SimulationManager = _SM
    monkeypatch.setitem(sys.modules, "isaacsim.core.simulation_manager", fake_sm_mod)
    monkeypatch.setitem(sys.modules, "isaacsim.core.version",
                        types.SimpleNamespace(get_version=lambda: "6.0.0"))

    import importlib
    import isaac_sim_mcp_extension.adapters.v6 as v6_mod
    importlib.reload(v6_mod)
    adapter = v6_mod.IsaacAdapterV6()
    adapter._ensure_physics_world()
    assert ("setup_simulation", 1.0 / 60.0) in sm_calls
    assert ("initialize_physics",) in sm_calls
    # And confirm we picked up the engine
    assert adapter._engine == "newton"
