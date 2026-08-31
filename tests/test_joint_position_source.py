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
# Copyright (c) 2024 whats2000
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

"""A joint read must say whether it measured physics or echoed a command.

When the physics view cannot serve a read, both adapters fall back to the
authored USD drive targets — the values set_joint_positions just wrote. Live on
Newton, an arm that was actually oscillating between -0.70 and -4.07 rad
reported exactly the commanded -0.400 / -2.000 through that fallback, which
reads as a perfectly converged robot. The response has to carry its provenance.
"""

import ast
import os

from isaac_sim_mcp_extension.handlers import robots as robots_handler

ADAPTERS = os.path.join(
    os.path.dirname(__file__), "..", "isaac.sim.mcp_extension", "isaac_sim_mcp_extension", "adapters"
)


class _Adapter:
    JOINT_SOURCE_PHYSICS = "physics"
    JOINT_SOURCE_DRIVE_TARGETS = "drive_targets"

    def __init__(self, source):
        self.joint_position_source = source

    def get_joint_positions(self, prim_path):
        return [0.0, -0.4, 0.0, -2.0]


def test_physics_backed_read_is_labelled_and_carries_no_warning():
    result = robots_handler.get_joints(_Adapter("physics"), prim_path="/World/Arm")
    assert result["status"] == "success"
    assert result["position_source"] == "physics"
    assert "warning" not in result


def test_drive_target_fallback_is_labelled_and_warns():
    result = robots_handler.get_joints(_Adapter("drive_targets"), prim_path="/World/Arm")
    assert result["status"] == "success"
    assert result["position_source"] == "drive_targets"
    assert "warning" in result, "an echoed command must not pass for a measurement"
    assert "not simulated positions" in result["warning"]


def _fallback_is_tagged(filename):
    """The USD-drive-target branch of get_joint_positions must record its source."""
    with open(os.path.join(ADAPTERS, filename)) as f:
        text = f.read()
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "get_joint_positions":
            src = ast.get_source_segment(text, node) or ""
            return "JOINT_SOURCE_DRIVE_TARGETS" in src and "JOINT_SOURCE_PHYSICS" in src
    return False


def test_v5_get_joint_positions_records_which_branch_answered():
    assert _fallback_is_tagged("v5.py")


def test_v6_get_joint_positions_records_which_branch_answered():
    assert _fallback_is_tagged("v6.py")
