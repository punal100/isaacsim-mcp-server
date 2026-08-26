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

"""Invariants around when physics is first initialised.

Every assertion here stands for a measured simulator failure, so weakening one
re-opens the bug it names. See CHANGELOG for the measurements.
"""

import ast
import os

EXT = os.path.join(os.path.dirname(__file__), "..", "isaac.sim.mcp_extension", "isaac_sim_mcp_extension")


def _source(*parts):
    with open(os.path.join(EXT, *parts)) as f:
        return f.read()


def _function_src(text, name):
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(text, node) or ""
    raise AssertionError(f"function {name!r} not found")


def test_create_physics_scene_primes_physics_before_any_robot_exists():
    """PhysX corrupts its GPU pipeline when physics comes up after the first robot.

    Measured on 6.0.1-rc.7, two FR3s created through the tools: initialising
    physics before either robot steps cleanly, initialising after the first and
    then adding the second dies with "PhysX Internal CUDA error ... Error code
    700" plus one PhysX ABORT per stepped frame. step_simulation still returns
    success while physics is dead, so nothing surfaces the failure.
    """
    src = _function_src(_source("handlers", "scene.py"), "create_physics")
    assert "_ensure_physics_world()" in src


def test_create_physics_scene_does_not_prime_newton():
    """Newton builds its model when physics comes up, so priming it early freezes it.

    Measured on 6.0.1-rc.7 under the Newton kit: with the PhysX priming applied
    unconditionally, a rigid-body-only stage stopped simulating entirely -- a
    sphere dropped from z=2 stayed at 2.000 where it otherwise lands at 0.149.
    Scenes containing a robot kept working, which is why this needs a test.
    """
    src = _function_src(_source("handlers", "scene.py"), "create_physics")
    assert "newton" in src.lower(), "the priming must stay behind an engine check"


def test_newton_model_rebuild_runs_the_geometry_guard():
    """The rebuild calls initialize_newton() directly and can latch physics dead.

    A cone or a zero-sized shape makes Newton's MuJoCo model builder raise;
    Isaac catches it and latches NewtonStage._init_failed permanently, after
    which only a restart recovers. The guard exists to refuse *before* that
    happens, so every path that initialises Newton has to run it.
    """
    src = _function_src(_source("adapters", "v6.py"), "_refresh_newton_model_if_stale")
    assert "_guard_newton_unsupported_geometry()" in src


def test_newton_guard_covers_zero_sized_shapes():
    """ "Only plane shapes are allowed to have a size of zero" latches like the cone."""
    text = _source("adapters", "v6.py")
    assert "NEWTON_SIZE_ATTRS" in text
    src = _function_src(text, "_newton_unsupported_geometry")
    assert "_newton_zero_sized" in src


def test_step_rejects_a_non_positive_count():
    """A negative count silently did nothing on V5/Newton and errored on V6/PhysX.

    `for _ in range(num_steps)` is an empty loop, so the tool reported
    "Stepped -5 frames" for a call that advanced no physics at all.
    """
    src = _function_src(_source("handlers", "simulation.py"), "step")
    assert "num_steps < 1" in src


def test_create_object_rejects_a_size_that_scales_the_prim_to_nothing():
    """size becomes a scale factor, so size<=0 renders and collides as nothing.

    On Newton it also latches physics dead for the rest of the session.
    """
    src = _function_src(_source("handlers", "objects.py"), "create")
    assert "size <= 0" in src


def test_capture_image_checks_the_camera_before_building_the_rtx_pipeline():
    """Capturing a path that does not exist used to author one.

    A single capture_image on a typo'd path created a Camera prim there plus a
    render product and a five-node SDG OmniGraph, then asked Replicator to
    render a frame for it. That is the stray camera + render product pair that
    cannot be reliably deleted, and on Newton it broke stepping outright — a
    sphere dropped from z=2 froze at 1.992 through 180 steps where it lands at
    0.149. The check has to come before adapter.capture_camera_image.
    """
    text = _source("handlers", "sensors.py")
    src = _function_src(text, "capture_image")
    assert "prim_missing" in src
    assert src.index("prim_missing") < src.index("capture_camera_image")


def test_lidar_read_checks_the_sensor_prim_first():
    """get_lidar_point_cloud builds a LidarSensor wrapper for whatever path it gets."""
    src = _function_src(_source("handlers", "sensors.py"), "get_point_cloud")
    assert "prim_missing" in src
    assert src.index("prim_missing") < src.index("get_lidar_point_cloud")
