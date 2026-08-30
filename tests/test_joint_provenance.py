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

"""Joint reads must say whether they measured physics or echoed the command.

get_joint_positions says so. The two other paths that read joints do not:

  * step(observe_joints=...) — which the server instructions call *the* debug
    loop, so it is the read an agent leans on most.
  * get_joint_config — worse, because it computes
    position_error = target - actual, and when the echo answers, actual *is*
    the target. It prints exactly 0.0, which reads as perfectly converged at
    the precise moment the read is least trustworthy. On Newton, where drives
    genuinely diverge (#21), that inverts the diagnosis.
"""


class _EchoAdapter:
    """Adapter whose joint read fell back to drive targets."""

    JOINT_SOURCE_PHYSICS = "physics"
    JOINT_SOURCE_DRIVE_TARGETS = "drive_targets"

    def __init__(self, source):
        self._source = source

    @property
    def joint_position_source(self):
        return self._source

    def get_joint_config(self, prim_path):
        return {
            "prim_path": prim_path,
            "joint_count": 1,
            "joints": [
                {
                    "name": "j1",
                    "target_position": -0.4,
                    "actual_position": -0.4,
                    "position_error": 0.0,
                }
            ],
        }

    def get_simulation_state(self):
        return {"timeline_state": "stopped"}

    def step(self, num_steps=1, observe_prims=None, observe_joints=None):
        return {
            "stepped": num_steps,
            "joint_states": [{"prim_path": "/World/Arm", "joints": {"j1": -0.4}}],
        }


def test_joint_config_flags_an_echoed_read():
    from isaac_sim_mcp_extension.handlers.simulation import get_joint_config_handler

    out = get_joint_config_handler(_EchoAdapter("drive_targets"), prim_path="/World/Arm")

    assert out["position_source"] == "drive_targets"
    assert "warning" in out, "an echoed joint_config must say so"


def test_joint_config_does_not_report_a_zero_error_from_an_echo():
    """position_error = target - actual is identically 0 when actual is the
    target. Reporting it as a measurement is the wrong answer, not a small one."""
    from isaac_sim_mcp_extension.handlers.simulation import get_joint_config_handler

    out = get_joint_config_handler(_EchoAdapter("drive_targets"), prim_path="/World/Arm")

    assert "position_error" not in out["joints"][0], (
        "position_error was kept from an echoed read, where it is 0.0 by construction and reads as converged"
    )


def test_joint_config_keeps_the_error_when_physics_answered():
    from isaac_sim_mcp_extension.handlers.simulation import get_joint_config_handler

    out = get_joint_config_handler(_EchoAdapter("physics"), prim_path="/World/Arm")

    assert out["position_source"] == "physics"
    assert "position_error" in out["joints"][0]
    assert "warning" not in out


def test_step_observe_joints_flags_an_echoed_read():
    """The debug loop's own read has to carry the same tag."""
    from isaac_sim_mcp_extension.handlers.simulation import step

    out = step(_EchoAdapter("drive_targets"), num_steps=5, observe_joints=["/World/Arm"])

    assert out["position_source"] == "drive_targets"
    assert "warning" in out


def test_step_without_observed_joints_is_untagged():
    """No joints read, nothing to qualify — the tag would be noise."""
    from isaac_sim_mcp_extension.handlers.simulation import step

    out = step(_EchoAdapter("drive_targets"), num_steps=5)

    assert "position_source" not in out
