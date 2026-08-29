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

"""release_sensor must actually tear a wrapper down, on every wrapper shape.

Introspected on 5.1.0: LidarRtx has no destroy() and no detach_annotators().
It has detach_writer(writer_name) -- which *requires* an argument, so calling it
with none raises TypeError straight into release_sensor's `except Exception:
pass`. The release therefore did nothing at all for a lidar: measured, the
annotator was still attached afterwards (1 attached, 0 after
detach_all_annotators), leaving its render product live for the life of the Kit
process.
"""

from isaac_sim_mcp_extension.adapters.base import IsaacAdapterBase


class _LidarRtxShaped:
    """The 5.1 LidarRtx surface, as introspected on the real class."""

    def __init__(self):
        self.calls = []
        self.annotators_attached = 1

    def detach_all_annotators(self):
        self.calls.append("detach_all_annotators")
        self.annotators_attached = 0

    def detach_all_writers(self):
        self.calls.append("detach_all_writers")

    def detach_annotator(self, annotator_name: str):
        self.calls.append(f"detach_annotator({annotator_name})")

    def detach_writer(self, writer_name: str):
        self.calls.append(f"detach_writer({writer_name})")


class _CameraShaped:
    """The 5.1 Camera surface: destroy() exists, and must stay the one used."""

    def __init__(self):
        self.calls = []

    def destroy(self):
        self.calls.append("destroy")

    def detach_annotator(self, annotator_name: str):
        self.calls.append(f"detach_annotator({annotator_name})")


class _Adapter(IsaacAdapterBase):
    """Concrete enough to exercise release_sensor without a stage."""

    def __init__(self):
        self._camera_sensors = {}
        self._lidar_sensors = {}

    def __getattr__(self, name):
        # The ABC declares many abstract methods this test never touches.
        raise AttributeError(name)

    def get_stage(self):
        return None


# IsaacAdapterBase is abstract; only release_sensor is under test here.
_Adapter.__abstractmethods__ = frozenset()


def test_lidar_wrapper_is_actually_torn_down():
    adapter = _Adapter()
    lidar = _LidarRtxShaped()
    adapter._lidar_sensors["/World/L"] = lidar

    adapter.release_sensor("/World/L")

    assert lidar.annotators_attached == 0, (
        "release_sensor left the annotator attached; its render product keeps rendering for the life of the process"
    )
    assert "detach_all_annotators" in lidar.calls


def test_camera_still_uses_destroy():
    """5.1's Camera does have destroy(); the lidar fix must not regress it."""
    adapter = _Adapter()
    camera = _CameraShaped()
    adapter._camera_sensors["/World/C"] = camera

    adapter.release_sensor("/World/C")

    assert "destroy" in camera.calls


def test_release_never_calls_a_detach_that_needs_an_argument():
    """detach_writer/detach_annotator take a name; calling them bare raises
    TypeError, which the bare except swallows -- that is how a release that did
    nothing looked like one that worked."""
    adapter = _Adapter()
    lidar = _LidarRtxShaped()
    adapter._lidar_sensors["/World/L"] = lidar

    adapter.release_sensor("/World/L")

    for call in lidar.calls:
        assert not call.startswith("detach_writer("), f"called arg-taking {call}"
        assert not call.startswith("detach_annotator("), f"called arg-taking {call}"


def test_the_sensor_is_forgotten_either_way():
    adapter = _Adapter()
    adapter._lidar_sensors["/World/L"] = _LidarRtxShaped()

    adapter.release_sensor("/World/L")

    assert "/World/L" not in adapter._lidar_sensors


class _V6SensorShaped:
    """6.0's _SensorRuntime surface, as read from the installed sources.

    CameraSensor and LidarSensor both extend _SensorRuntime. It has no
    destroy(), no detach_all_annotators() and no detach_all_writers(); the only
    zero-argument teardown is _invalidate_sensor(), which is what its own
    __del__ calls. detach_annotators(self, annotators) requires an argument, so
    calling it bare raises TypeError -- the same trap 5.1's detach_writer set.
    """

    def __init__(self):
        self.invalidated = False
        self.bad_calls = []

    def detach_annotators(self, annotators):
        self.bad_calls.append("detach_annotators")

    def detach_writer(self, writer_name):
        self.bad_calls.append("detach_writer")

    def _invalidate_sensor(self):
        self.invalidated = True


def test_v6_sensor_is_actually_torn_down():
    """Without this, release_all_sensors on every timeline STOP frees nothing on
    6.0, and only garbage collection eventually runs __del__ -- which is why the
    leak looked intermittent."""
    adapter = _Adapter()
    sensor = _V6SensorShaped()
    adapter._camera_sensors["/World/C"] = sensor

    adapter.release_sensor("/World/C")

    assert sensor.invalidated, "release_sensor did nothing for a 6.0 sensor"


def test_methods_needing_arguments_are_never_called_bare():
    """The invariant was a comment saying every name must take no arguments, and
    a comment did not stop detach_annotators being added to the list. Enforce it
    by inspecting the signature instead."""
    adapter = _Adapter()
    sensor = _V6SensorShaped()
    adapter._camera_sensors["/World/C"] = sensor

    adapter.release_sensor("/World/C")

    assert sensor.bad_calls == [], f"called methods that require arguments: {sensor.bad_calls}"
