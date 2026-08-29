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

"""Creating a robot on Newton must say that its drives will not converge.

Issue #21 is engine-level and cannot be fixed here: the MuJoCo actuator gain
cannot be integrated at the configured timestep, and the GPU path runs
`mjw_model` rather than the `mj_model` copy this code can reach. Measured, the
FR3 sails past a -0.4 target to -0.736 by 1200 steps and keeps going, while
PhysX settles at -0.398 within 150.

An unfixable behaviour still has to be signalled, or an agent watches an
articulation diverge with nothing to tell it that is expected — the same reason
create_camera warns about #20 and delete_object warns about a resurrected
sensor prim.
"""

import ast
import os


class _Engine:
    """Only what the warning helper reads."""

    def __init__(self, engine):
        if engine is not None:
            self._engine = engine


def test_newton_is_warned_about_drive_divergence():
    from isaac_sim_mcp_extension.handlers.robots import engine_drive_warning

    warning = engine_drive_warning(_Engine("newton"))

    assert warning, "Newton must be flagged"
    assert "converge" in warning
    assert "PhysX" in warning, "the warning should name the engine that does converge"
    assert "limit" in warning, "limits are unenforced too, and that surprises people"


def test_physx_is_not_warned():
    from isaac_sim_mcp_extension.handlers.robots import engine_drive_warning

    assert engine_drive_warning(_Engine("physx")) is None


def test_v5_is_not_warned():
    """5.1 has no _engine attribute at all and converges normally."""
    from isaac_sim_mcp_extension.handlers.robots import engine_drive_warning

    assert engine_drive_warning(_Engine(None)) is None


def test_create_robot_appends_it_alongside_existing_warnings():
    """The zero-stiffness drive warning must survive next to it."""
    src = os.path.join(
        os.path.dirname(__file__),
        "..",
        "isaac.sim.mcp_extension",
        "isaac_sim_mcp_extension",
        "handlers",
        "robots.py",
    )
    with open(src) as f:
        tree = ast.parse(f.read())

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "create":
            called = {n.func.id for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
            assert "engine_drive_warning" in called, "create() never asks about the engine"
            # appended, not assigned over: the joint-config warnings must remain
            appends = [
                n
                for n in ast.walk(node)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "append"
            ]
            assert appends, "the engine warning should be appended to the existing list"
            return
    raise AssertionError("create() not found")
