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

"""6.0 lidar output is spherical and must be converted before it is returned.

Measured on 6.0.1 (both engines): the prim carries
`omni:sensor:Core:elementsCoordsType = SPHERICAL`, so GenericModelOutput's
x/y/z are azimuth degrees, elevation degrees and range metres. They were
returned unconverted under a "sensor-local coordinates, meters" label, so
`bounds` and `nearest` were computed over angles. See issue #22.
"""

import ast
import math
import os

from isaac_sim_mcp_extension.adapters.base import spherical_to_cartesian

ADAPTERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
)


def _close(a, b, tol=1e-6):
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def test_straight_ahead_hit_maps_to_the_x_axis():
    """A return dead ahead at 3 m is 3 m along +X, not 3 along Z."""
    out = spherical_to_cartesian([0.0], [0.0], [3.0])
    assert _close(out[0], [3.0, 0.0, 0.0])


def test_azimuth_ninety_degrees_maps_to_the_y_axis():
    out = spherical_to_cartesian([90.0], [0.0], [2.0])
    assert _close(out[0], [0.0, 2.0, 0.0])


def test_elevation_is_measured_from_the_horizontal_plane():
    """Straight up is +Z; the sensor's -15 deg floor returns must come back below it."""
    up = spherical_to_cartesian([0.0], [90.0], [5.0])
    assert _close(up[0], [0.0, 0.0, 5.0])

    down = spherical_to_cartesian([0.0], [-15.0], [4.0])
    assert down[0][2] < 0, "a negative elevation must map below the sensor"
    assert math.isclose(down[0][2], 4.0 * math.sin(math.radians(-15)), rel_tol=1e-9)


def test_range_is_preserved_as_euclidean_distance():
    """The whole point: nearest.distance must equal the measured range again."""
    az = [12.0, -30.0, 175.0]
    el = [-7.0, 3.5, 9.0]
    rng = [3.0, 9.0, 21.5]

    out = spherical_to_cartesian(az, el, rng)

    for row, expected in zip(out, rng):
        assert math.isclose(math.dist([0, 0, 0], row), expected, rel_tol=1e-9)


def test_measured_wall_geometry_round_trips():
    """The live reproduction from #22: a 2 m wall whose near face is 3 m ahead.

    Its edge subtends atan(1/3) = 18.43 deg, which is exactly the azimuth
    spread that was being reported as a Cartesian X of +-18.41.
    """
    edge_az = math.degrees(math.atan(1 / 3))
    out = spherical_to_cartesian([edge_az], [0.0], [3.0 / math.cos(math.radians(edge_az))])
    assert math.isclose(out[0][0], 3.0, rel_tol=1e-9), "wall face should sit 3 m along X"
    assert math.isclose(out[0][1], 1.0, rel_tol=1e-9), "wall edge should sit 1 m off-axis"


def test_empty_sweep_returns_no_points():
    assert spherical_to_cartesian([], [], []) == []


def test_v6_converts_when_the_prim_declares_spherical():
    """v6 must consult elementsCoordsType rather than assuming either layout."""
    with open(os.path.join(ADAPTERS, "v6.py")) as f:
        src = f.read()
    assert "elementsCoordsType" in src, "v6 never checks the sensor's coordinate convention"
    tree = ast.parse(src)
    called = {n.func.id for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert "spherical_to_cartesian" in called, "v6 returns the annotator's values unconverted"
