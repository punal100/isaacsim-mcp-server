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


def test_51_lidar_is_deliberately_not_torn_down():
    """5.1's LidarRtx teardown works and is deliberately not called.

    detach_all_annotators() does detach — 1 attached before, 0 after — but
    calling it hangs Kit. Measured on 5.1.0 from a cold boot, looping
    create_lidar + clear_scene: wedged on round 0 with it (twice, in two
    different sequences), against round 4 in one run and no wedge in five in
    another without it. A pre-existing intermittent hang becomes immediate and
    reliable.

    It buys nothing to pay for that. The 5.1 lidar prim survives clear_scene
    regardless (#25), its render products leak either way, and a fresh path
    reads 33% with or without it (#31) — the fill-rate gain once credited to
    this call was really the poisoned-path fix.

    Revisit if the underlying hang is ever fixed upstream.
    """
    adapter = _Adapter()
    lidar = _LidarRtxShaped()
    adapter._lidar_sensors["/World/L"] = lidar

    adapter.release_sensor("/World/L")

    assert lidar.calls == [], f"a 5.1 lidar teardown was called: {lidar.calls}"
    assert "/World/L" not in adapter._lidar_sensors, "the wrapper must still be forgotten"


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


def test_the_guard_itself_classifies_teardown_methods():
    """Assert on the guard, not on a side effect it cannot produce.

    Watching `bad_calls` stay empty proves nothing: an arg-taking method raises
    TypeError before its body runs, so the list is empty whether or not the
    guard exists — verified by deleting the guard and watching the test still
    pass. The invariant has to be asserted directly, because it is the thing
    that failed twice.
    """
    from isaac_sim_mcp_extension.adapters.base import _needs_arguments

    sensor = _V6SensorShaped()

    assert _needs_arguments(sensor.detach_annotators), "detach_annotators(annotators) must be skipped"
    assert _needs_arguments(sensor.detach_writer), "detach_writer(writer_name) must be skipped"
    assert not _needs_arguments(sensor._invalidate_sensor), "_invalidate_sensor() must be callable"


def test_guard_allows_optional_arguments():
    """A default-valued parameter is still callable bare and must not be skipped."""
    from isaac_sim_mcp_extension.adapters.base import _needs_arguments

    class _Optional:
        def destroy(self, force=False):
            pass

    assert not _needs_arguments(_Optional().destroy)


def test_guard_calls_when_the_signature_cannot_be_read():
    """Unreadable signature means call it, not skip it.

    Some Isaac wrappers are C-implemented and inspect.signature raises on them.
    Skipping such a method reintroduces exactly the silent no-op this guard
    exists to prevent, whereas calling one that turns out to need arguments
    costs a TypeError the call site already swallows. Bias toward calling.

    These two cover both arms of the guard's `except (TypeError, ValueError)`:
    inspect.signature(print) raises ValueError, inspect.signature(object())
    raises TypeError.
    """
    from isaac_sim_mcp_extension.adapters.base import _needs_arguments

    assert not _needs_arguments(print)  # ValueError arm
    assert not _needs_arguments(object())  # TypeError arm
