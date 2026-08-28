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

"""Object creation and manipulation command handlers."""

from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

from ..adapters.base import IsaacAdapterBase

# USD primitive default dimensions in meters. Used to translate the friendly
# `size` parameter (target size in meters) into the scale factor needed to
# produce an object of that size. Without this, naive callers get the USD
# defaults — a 2m³ Cube, a 2m-diameter Sphere — which are surprising next
# to a typical 1.2m-tall robot.
_USD_DEFAULT_SIZE_M: Dict[str, float] = {
    "Cube": 2.0,  # UsdGeom.Cube default size = 2
    "Sphere": 2.0,  # UsdGeom.Sphere default radius = 1 → diameter 2
    "Cylinder": 2.0,  # UsdGeom.Cylinder default height = 2
    "Cone": 2.0,  # UsdGeom.Cone default height = 2
    "Capsule": 2.0,  # UsdGeom.Capsule height=1 + 2*radius(0.5) = 2 end-to-end
    "Plane": 1.0,  # UsdGeomPlane default width/length = 1
}

# Case-insensitive map from any casing to the canonical USD type name. USD type
# names are case-sensitive, so passing "cube" to create_prim silently produces a
# typeless prim with no geometry (no actual_size). Normalising here lets callers
# (and agents) pass "cube"/"CUBE" and still get a real UsdGeom.Cube.
_CANONICAL_PRIM_TYPES: Dict[str, str] = {name.lower(): name for name in _USD_DEFAULT_SIZE_M}


def _unregistered_type(adapter: IsaacAdapterBase, prim_path: str):
    """The authored type name when USD has no schema for it, else None.

    USD does not reject an unknown type name — it authors the prim with that
    name and no schema behind it, so create_object(object_type="NotAShape")
    reported "Created NotAShape" for something that never renders or collides.
    Measured on 5.1: the bogus prim reads authored='NotAShape' schema='' while a
    real one reads authored='Cube' schema='Cube'. Returns None whenever the
    answer cannot be read, so only a definite mismatch is ever rejected.
    """
    try:
        prim = adapter.get_stage().GetPrimAtPath(prim_path)
        if not (prim and prim.IsValid()):
            return None
        authored = prim.GetTypeName()
        schema = prim.GetPrimTypeInfo().GetSchemaTypeName()
    except Exception:
        return None
    try:
        authored_s, schema_s = str(authored), str(schema)
    except Exception:
        return None
    # A stubbed/mocked stage yields non-empty placeholder text for both; only
    # a real, empty schema name means USD did not recognise the type.
    if authored_s and not schema_s:
        return authored_s
    return None


def prim_missing(adapter: IsaacAdapterBase, prim_path: str) -> bool:
    """True when prim_path is not on the stage.

    Handlers that act on a prim have to check this themselves: the adapters
    answer a missing prim with empty data rather than an exception, so an
    unchecked handler reports success for a path that was never there — a typo
    in a prim path came back as {"status": "success"} with nothing done.
    """
    try:
        prim = adapter.get_stage().GetPrimAtPath(prim_path)
    except Exception:
        return False  # cannot tell; let the operation speak for itself
    return not (prim and prim.IsValid())


def register(registry: Dict[str, Any], adapter: IsaacAdapterBase) -> None:
    registry["objects.create"] = lambda **p: create(adapter, **p)
    registry["objects.delete"] = lambda **p: delete(adapter, **p)
    registry["objects.transform"] = lambda **p: transform(adapter, **p)
    registry["objects.clone"] = lambda **p: clone(adapter, **p)


def create(
    adapter: IsaacAdapterBase,
    object_type: str = "Cube",
    position: Optional[Sequence[float]] = None,
    rotation: Optional[Sequence[float]] = None,
    scale: Optional[Sequence[float]] = None,
    size: Optional[float] = None,
    color: Optional[Sequence[float]] = None,
    physics_enabled: bool = False,
    prim_path: Optional[str] = None,
) -> Dict[str, Any]:
    try:
        # Normalise to the canonical USD type name so non-canonical casing
        # ("cube") still creates real geometry instead of a typeless prim.
        object_type = _CANONICAL_PRIM_TYPES.get(object_type.lower(), object_type)

        # size becomes a scale factor below, so size<=0 authors a prim scaled to
        # zero (or mirrored): it renders and collides as nothing, exactly like
        # the unknown-type case handled further down. On Newton it is worse than
        # useless — the MuJoCo model builder refuses any non-plane shape of zero
        # size and latches physics dead for the rest of the session.
        if size is not None and size <= 0:
            return {
                "status": "error",
                "message": (
                    f"size must be greater than 0 (got {size}); a zero or negative size scales the "
                    "prim to nothing, so it would render and collide as nothing."
                ),
            }
        if not prim_path:
            stage = adapter.get_stage()
            count = len(list(stage.TraverseAll()))
            prim_path = f"/World/{object_type}_{count}"
        _prim = adapter.create_prim(prim_path, prim_type=object_type)

        # USD does not reject an unknown type name — it authors a *typeless*
        # prim, so create_object(object_type="NotAShape") reported
        # "Created NotAShape" for something that never renders or collides.
        # Types outside _CANONICAL_PRIM_TYPES are still allowed (Xform and
        # friends are legitimate); the test is whether USD kept the type.
        created_type = _unregistered_type(adapter, prim_path)
        if created_type is not None:
            try:
                adapter.delete_prim(prim_path)
            except Exception:
                pass
            return {
                "status": "error",
                "message": (
                    f"USD has no schema for object_type {object_type!r}, so the prim would render "
                    f"and collide as nothing. Geometric options: " + ", ".join(sorted(_CANONICAL_PRIM_TYPES.values()))
                ),
            }

        # When no scale was given, derive one from `size` (default 1m) so the
        # object comes out at a sane size relative to a typical robot. If the
        # caller passed an explicit scale, that wins — `size` is ignored.
        if scale is None:
            target_size = size if size is not None else 1.0
            default_dim = _USD_DEFAULT_SIZE_M.get(object_type, 2.0)
            factor = target_size / default_dim
            scale = [factor, factor, factor]

        if position or rotation or scale:
            adapter.set_prim_transform(prim_path, position=position, rotation=rotation, scale=scale)

        # All objects get collision so they interact with the scene.
        # physics_enabled additionally adds RigidBodyAPI for dynamic simulation.
        from pxr import UsdPhysics

        stage = adapter.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            if physics_enabled and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(prim)

        response: Dict[str, Any] = {"status": "success", "message": f"Created {object_type}", "prim_path": prim_path}
        try:
            actual_size, (bbox_min, bbox_max) = adapter.get_prim_actual_size(prim_path)
            response["actual_size"] = actual_size
            response["bounding_box"] = {"min": bbox_min, "max": bbox_max}
        except Exception:
            pass
        return response
    except Exception as e:
        return {"status": "error", "message": str(e)}


def delete(adapter: IsaacAdapterBase, prim_path: Optional[str] = None) -> Dict[str, Any]:
    try:
        if not prim_path:
            return {"status": "error", "message": "prim_path is required"}
        # Deleting something that was never there used to report success — the
        # post-delete "is it gone?" check below is trivially true for a path
        # that never existed, so a typo read as a successful delete.
        if prim_missing(adapter, prim_path):
            return {"status": "error", "message": f"Prim not found: {prim_path}"}

        # Note the type before it goes: RTX sensors are the prims that come back.
        doomed_type = ""
        try:
            stage_before = adapter.get_stage()
            if stage_before is not None:
                doomed = stage_before.GetPrimAtPath(prim_path)
                if doomed and doomed.IsValid():
                    doomed_type = str(doomed.GetTypeName())
        except Exception:
            pass

        adapter.delete_prim(prim_path)

        # Confirm it actually went, so an immediate failure is not reported as
        # success. This cannot catch every case: an RTX camera on 6.0 is gone in
        # this tick and back in the next, because its RtxCamera wrapper has no
        # teardown method (only reset_to_default_state /
        # reset_xform_op_properties / valid), Isaac holds it internally, and it
        # re-creates the prim -- which reappears at the end of the parent's
        # children with its render product still targeting it. A handler cannot
        # wait a tick to check, so that case is documented in the changelog
        # rather than detected here.
        stage = adapter.get_stage()
        if stage is not None:
            survivor = stage.GetPrimAtPath(prim_path)
            if survivor and survivor.IsValid():
                is_camera = survivor.GetTypeName() == "Camera"
                detail = (
                    " On Isaac Sim 6.0 an RTX camera may also reappear a tick later; "
                    "reuse the camera instead of deleting it."
                    if is_camera
                    else " Something still holds it; check for a live sensor or reference."
                )
                return {
                    "status": "error",
                    "message": f"{prim_path} still exists after delete.{detail}",
                    "prim_path": prim_path,
                }
        result = {"status": "success", "message": f"Deleted {prim_path}"}
        if doomed_type in ("OmniLidar", "Camera"):
            # The check above is same-tick, and an RTX sensor is gone in this
            # tick and back in the next: measured on 5.1.0, deleting an
            # OmniLidar leaves no OmniLidar on the stage but puts a Camera prim
            # at the same path a tick later, and on 6.0 the session's first
            # camera returns in full (#20). A handler cannot wait a tick to look
            # again -- pumping Kit's loop from inside one crashes it -- so say
            # the resurrection is possible rather than let plain success imply
            # the path is now free.
            result["warning"] = (
                f"{prim_path} was an RTX sensor ({doomed_type}) and Replicator may re-create a prim "
                "at this path on the next tick — the delete itself succeeded, but the path may not "
                "stay free. Confirm with list_prims before reusing it, and prefer reusing a sensor "
                "over deleting and re-creating one."
            )
        return result
    except Exception as e:
        return {"status": "error", "message": str(e)}


def transform(
    adapter: IsaacAdapterBase,
    prim_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    rotation: Optional[Sequence[float]] = None,
    scale: Optional[Sequence[float]] = None,
) -> Dict[str, Any]:
    try:
        if not prim_path:
            return {"status": "error", "message": "prim_path is required"}
        adapter.set_prim_transform(prim_path, position=position, rotation=rotation, scale=scale)
        return {"status": "success", "message": f"Transformed {prim_path}"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def clone(
    adapter: IsaacAdapterBase,
    source_path: Optional[str] = None,
    target_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
) -> Dict[str, Any]:
    try:
        if not source_path or not target_path:
            return {"status": "error", "message": "source_path and target_path are required"}
        if prim_missing(adapter, source_path):
            return {"status": "error", "message": f"Source prim not found: {source_path}"}
        adapter.clone_prim(source_path, target_path)
        # CopyPrim does not raise for a source it cannot copy, so confirm the
        # clone is actually on the stage before calling it a success.
        if prim_missing(adapter, target_path):
            return {"status": "error", "message": f"Clone produced no prim at {target_path}"}
        if position:
            adapter.set_prim_transform(target_path, position=position)
        return {"status": "success", "message": f"Cloned {source_path} to {target_path}"}
    except Exception as e:
        return {"status": "error", "message": str(e)}
