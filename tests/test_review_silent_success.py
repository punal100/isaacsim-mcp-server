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

"""Paths that reported success for work that did not happen.

Each is the class 42072b2 set out to eliminate — a parameter accepted and
dropped, or a success reported for a thing that never took effect.
"""


# ── create_lidar(config=...) on 6.0 ─────────────────────────────────────────


class _V6LidarAdapter:
    SUPPORTS_LIDAR_CONFIG = False
    _engine = "physx"

    def __init__(self):
        self.created = []

    def get_stage(self):
        class _S:
            def GetPrimAtPath(self, p):
                class _P:
                    def IsValid(self):
                        return False

                    def GetTypeName(self):
                        return ""

                return _P()

        return _S()

    def create_lidar(self, prim_path, config=None, **kw):
        self.created.append((prim_path, config))
        return object()

    def set_prim_transform(self, prim_path, **kw):
        return True


class _V5LidarAdapter(_V6LidarAdapter):
    SUPPORTS_LIDAR_CONFIG = True
    _engine = None


def test_lidar_config_ignored_on_60_is_reported():
    """Measured on 6.0.1: a lidar created with config="Example_Rotary" and one
    created with no config are identical — model=LidarCore, channels=128. The
    6.0 constructor takes only a path; presets are schema attributes applied
    afterwards. Asking for a hardware model and silently getting a generic
    sensor is the wrong answer, not a lesser one."""
    from isaac_sim_mcp_extension.handlers.sensors import create_lidar

    out = create_lidar(_V6LidarAdapter(), prim_path="/World/L", config="Example_Rotary")

    assert out["status"] == "success"
    assert "warning" in out, "an ignored config must be reported"
    assert "Example_Rotary" in out["warning"]


def test_no_warning_when_no_config_was_asked_for():
    from isaac_sim_mcp_extension.handlers.sensors import create_lidar

    out = create_lidar(_V6LidarAdapter(), prim_path="/World/L")

    assert "warning" not in out


def test_no_warning_where_the_config_is_honoured():
    from isaac_sim_mcp_extension.handlers.sensors import create_lidar

    out = create_lidar(_V5LidarAdapter(), prim_path="/World/L", config="Example_Rotary")

    assert "warning" not in out


# ── create_robot with an asset that did not resolve ──────────────────────────


class _RobotAdapter:
    _engine = None

    def __init__(self, num_dof, joint_names=None, raises=False):
        self._num_dof, self._names, self._raises = num_dof, joint_names or [], raises

    def get_assets_root_path(self):
        return "https://example.invalid"

    def discover_robots(self):
        return {"frankafr3": {"key": "frankafr3", "description": "FR3", "asset_path": "/x.usd", "manufacturer": "F"}}

    def add_reference_to_stage(self, asset_path, prim_path):
        return object()

    def set_prim_transform(self, prim_path, **kw):
        return True

    def get_robot_joint_info(self, prim_path):
        if self._raises:
            raise RuntimeError("articulation could not be read")
        return {"joint_names": self._names, "num_dof": self._num_dof}

    def get_joint_config(self, prim_path):
        return {"joints": [], "warnings": []}


def _create(adapter):
    from isaac_sim_mcp_extension.handlers import robots

    robots._discovered_robots = None
    return robots.create(adapter, robot_type="frankafr3", prim_path="/World/Arm")


def test_robot_with_zero_dof_is_flagged():
    """V6 does not raise for an unresolved asset — it falls back to a USD walk
    and returns num_dof 0, so a robot that is not there reads as a successful
    create."""
    out = _create(_RobotAdapter(num_dof=0))

    assert "warnings" in out
    assert any("0 joints" in w or "no joints" in w.lower() for w in out["warnings"])


def test_robot_whose_joint_read_raised_is_flagged():
    """The exception was printed to Kit's log and swallowed, so the response
    promised joint_names and num_dof and carried neither."""
    out = _create(_RobotAdapter(num_dof=9, raises=True))

    assert "warnings" in out
    assert any("joint" in w.lower() for w in out["warnings"])


def test_a_healthy_robot_is_not_flagged():
    out = _create(_RobotAdapter(num_dof=9, joint_names=[f"j{i}" for i in range(9)]))

    assert out["num_dof"] == 9
    assert not any("joint" in w.lower() for w in out.get("warnings", []))


# ── load_environment with unusable bounds ────────────────────────────────────


class _EnvAdapter:
    def __init__(self, bounds):
        self._bounds = bounds

    def get_assets_root_path(self):
        return "https://example.invalid"

    def discover_environments(self):
        return {"grid": {"asset_path": "/g.usd", "description": "Grid"}}

    def load_environment(self, env_path, prim_path="/Environment"):
        return None

    def get_stage(self):
        return None


def test_environment_without_bounds_says_so(monkeypatch):
    """bounds carry floor_height, which is what lets a caller place objects on
    the ground. Omitting them silently leaves the caller to guess z."""
    from isaac_sim_mcp_extension.handlers import scene

    monkeypatch.setattr(scene, "_reference_conversion", lambda *a, **k: None)
    monkeypatch.setattr(scene, "_world_bounds", lambda *a, **k: None)
    scene._discovered_envs = None

    out = scene.load_environment(_EnvAdapter(None), environment="grid")

    assert out["status"] == "success"
    assert "warning" in out, "an environment with no usable bounds must say so"
    assert "bounds" in out["warning"].lower()


def test_environment_with_bounds_is_not_flagged(monkeypatch):
    from isaac_sim_mcp_extension.handlers import scene

    monkeypatch.setattr(scene, "_reference_conversion", lambda *a, **k: None)
    monkeypatch.setattr(scene, "_world_bounds", lambda *a, **k: {"extent": [1, 1, 1], "floor_height": 0.0})
    scene._discovered_envs = None

    out = scene.load_environment(_EnvAdapter(None), environment="grid")

    assert "warning" not in out
    assert out["bounds"]["floor_height"] == 0.0


# ── edit_action_graph asked to change nothing ────────────────────────────────


def test_edit_action_graph_rejects_a_call_with_nothing_to_do():
    """It reported "Updated graph" for a call carrying neither values nor
    connections, so a caller that built its payload wrong was told it worked."""
    from isaac_sim_mcp_extension.handlers.graphs import edit_action_graph

    out = edit_action_graph(object(), graph_path="/World/G")

    assert out["status"] == "error"
    # Pin the specific refusal. `status == "error"` alone is satisfied by any
    # unrelated failure -- with the guard removed this call falls through to an
    # `import omni.graph` that also errors, which would look like a pass.
    assert "Nothing to change" in out["message"]
    assert "values" in out["message"] and "connections" in out["message"]
