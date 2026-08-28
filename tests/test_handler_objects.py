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

"""Behavioural tests for the objects command handler (mock adapter)."""

from unittest.mock import MagicMock

from isaac_sim_mcp_extension.handlers import objects

# create_object must normalize non-canonical type casing before create_prim.
# Verified live against Isaac Sim 6.0.1: object_type="cube" produced a typeless
# prim (type "cube", no actual_size); "Cube" produced a real UsdGeom.Cube.


def test_create_normalizes_lowercase_type_to_canonical():
    adapter = MagicMock()
    objects.create(adapter, object_type="cube", prim_path="/World/x")

    # create_prim is called first, before any later step can fail — assert it
    # received the canonical "Cube", not "cube".
    _args, kwargs = adapter.create_prim.call_args
    assert kwargs.get("prim_type") == "Cube"


def test_create_leaves_canonical_and_unknown_types_untouched():
    for given, expected in (("Cube", "Cube"), ("SPHERE", "Sphere"), ("Xform", "Xform")):
        adapter = MagicMock()
        objects.create(adapter, object_type=given, prim_path="/World/x")
        _args, kwargs = adapter.create_prim.call_args
        assert kwargs.get("prim_type") == expected


# ── deleting an RTX sensor (issues #20, #25) ─────────────────────────────────


class _SensorStage:
    """Stage whose prim vanishes on delete, the way an RTX sensor does in-tick."""

    def __init__(self, path, type_name):
        self._path = path
        self._type = type_name
        self.deleted = False

    def GetPrimAtPath(self, path):
        if path != self._path or self.deleted:
            return _GonePrim()
        return _LivePrim(self._type)


class _LivePrim:
    def __init__(self, type_name):
        self._type = type_name

    def IsValid(self):
        return True

    def GetTypeName(self):
        return self._type


class _GonePrim:
    def IsValid(self):
        return False

    def GetTypeName(self):
        return ""


class _SensorAdapter:
    def __init__(self, path, type_name):
        self.stage = _SensorStage(path, type_name)

    def get_stage(self):
        return self.stage

    def delete_prim(self, prim_path):
        self.stage.deleted = True
        return True


def test_deleting_a_lidar_warns_that_it_may_come_back():
    """Measured on 5.1.0: the OmniLidar is gone in-tick, and Replicator puts a
    Camera prim back at the same path a tick later. The handler's post-delete
    check runs in the same tick so it sees the prim gone and reports plain
    success, and the caller is left believing the path is free.

    A handler must not pump Kit's loop to wait a tick, so the honest move is to
    say the resurrection is possible and how to confirm it.
    """
    from isaac_sim_mcp_extension.handlers.objects import delete

    result = delete(_SensorAdapter("/World/L", "OmniLidar"), prim_path="/World/L")

    assert result["status"] == "success"
    assert "warning" in result, "an RTX sensor delete must say it can be undone a tick later"
    assert "list_prims" in result["warning"], "the warning should say how to confirm"


def test_deleting_a_camera_warns_too():
    from isaac_sim_mcp_extension.handlers.objects import delete

    result = delete(_SensorAdapter("/World/C", "Camera"), prim_path="/World/C")

    assert "warning" in result


def test_deleting_an_ordinary_prim_does_not_warn():
    """The warning must stay rare enough to mean something."""
    from isaac_sim_mcp_extension.handlers.objects import delete

    result = delete(_SensorAdapter("/World/Cube", "Cube"), prim_path="/World/Cube")

    assert result["status"] == "success"
    assert "warning" not in result
