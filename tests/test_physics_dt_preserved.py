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

"""_ensure_physics_world must not silently revert an authored physics rate.

It runs from roughly nine call sites -- every step, joint read and physics-state
query -- and called setup_simulation(dt=1/60) unconditionally. Measured on
6.0.1 Newton:

    after init                0.01667   (1/60)
    set_physics_dt(1/240)     0.00417   takes effect
    after ONE tool call       0.01667   reverted
    after a step              0.01667

So a caller who sets 240 Hz -- which is exactly what set_physics_params tells
them to do via execute_script, since no adapter implements time_step -- loses it
on their next tool call, with nothing said.
"""

import sys
import types


def _adapter(monkeypatch, calls, current_dt):
    """V6 adapter whose SimulationManager records the dt it is set up with."""

    class _SM:
        dt = current_dt

        @classmethod
        def get_active_physics_engine(cls):
            return "physx"

        @classmethod
        def _cleanup_stale_physics_scenes(cls):
            pass

        @classmethod
        def get_physics_dt(cls):
            return cls.dt

        @classmethod
        def setup_simulation(cls, dt=None, device=None):
            calls.append(dt)
            cls.dt = dt

        @classmethod
        def initialize_physics(cls):
            pass

    monkeypatch.setitem(sys.modules, "isaacsim.core.simulation_manager", types.SimpleNamespace(SimulationManager=_SM))
    monkeypatch.setitem(sys.modules, "isaacsim.core.version", types.SimpleNamespace(get_version=lambda: "6.0.1"))

    import importlib

    import isaac_sim_mcp_extension.adapters.v6 as v6_mod

    importlib.reload(v6_mod)
    adapter = v6_mod.IsaacAdapterV6()
    monkeypatch.setattr(adapter, "get_stage", lambda: object())
    monkeypatch.setattr(adapter, "_guard_newton_unsupported_geometry", lambda: None)
    return adapter


def test_an_authored_rate_survives_an_ordinary_tool_call(monkeypatch):
    calls = []
    adapter = _adapter(monkeypatch, calls, current_dt=1.0 / 240.0)

    adapter._ensure_physics_world()

    assert calls, "setup_simulation was never called"
    assert abs(calls[-1] - 1.0 / 240.0) < 1e-9, (
        f"an authored 240 Hz rate was reset to {1 / calls[-1]:.0f} Hz by an ordinary call"
    )


def test_the_default_rate_is_used_when_nothing_is_set(monkeypatch):
    calls = []
    adapter = _adapter(monkeypatch, calls, current_dt=None)

    adapter._ensure_physics_world()

    assert abs(calls[-1] - 1.0 / 60.0) < 1e-9, "should fall back to 60 Hz when no rate is set"


def test_a_nonsense_rate_falls_back_to_the_default(monkeypatch):
    calls = []
    adapter = _adapter(monkeypatch, calls, current_dt=0.0)

    adapter._ensure_physics_world()

    assert abs(calls[-1] - 1.0 / 60.0) < 1e-9, "a zero dt must not be propagated"
