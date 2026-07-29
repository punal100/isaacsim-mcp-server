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

"""create_physics must be safe to call twice."""

from unittest.mock import MagicMock

import pytest
from isaac_sim_mcp_extension.handlers import scene as scene_handlers


@pytest.fixture(autouse=True)
def _collision_api(monkeypatch):
    """The offline pxr stub has no UsdPhysics.CollisionAPI; supply one."""
    from pxr import UsdPhysics

    monkeypatch.setattr(UsdPhysics, "CollisionAPI", MagicMock(), raising=False)


class _FakePrim:
    def __init__(self, valid):
        self._valid = valid

    def IsValid(self):
        return self._valid

    def HasAPI(self, _api):
        return True


class _FakeStage:
    def __init__(self, existing):
        self.existing = set(existing)

    def GetPrimAtPath(self, path):
        return _FakePrim(str(path) in self.existing)


def _adapter(existing):
    a = MagicMock()
    a.create_physics_scene.return_value = "/World/PhysicsScene"
    stage = _FakeStage(existing)
    a.get_stage.return_value = stage
    # Creating the prim makes it exist, as it would on a real stage.
    a.create_prim.side_effect = lambda path, _type: stage.existing.add(str(path))
    return a


def test_ground_plane_created_when_missing():
    a = _adapter(existing=[])
    result = scene_handlers.create_physics(a)
    assert result["status"] == "success"
    a.create_prim.assert_called_once_with("/World/groundPlane", "Plane")


def test_second_call_does_not_recreate_ground_plane():
    """create_prim raises "A prim already exists" on a repeat call.

    The physics scene is established before the ground plane, so an unguarded
    create_prim made the tool report failure for work it had already done, and
    the message named groundPlane rather than anything the caller asked for.
    """
    a = _adapter(existing=["/World/groundPlane"])
    result = scene_handlers.create_physics(a)
    assert result["status"] == "success"
    a.create_prim.assert_not_called()
