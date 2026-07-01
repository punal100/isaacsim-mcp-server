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

"""Isaac Sim 6.0.0 adapter implementation (PhysX + Newton)."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .base import IsaacAdapterBase

if TYPE_CHECKING:
    from pxr import Usd


def _recompile_scriptnodes_for_file(abs_path: str) -> list:
    """Recompile every Action-Graph ScriptNode whose scriptPath matches abs_path.

    Returns the list of recompiled node paths (empty if none matched).
    """
    import os

    try:
        import omni.graph.core as og

        from ..handlers.graphs import force_recompile_scriptnode
    except Exception:
        return []

    recompiled = []
    try:
        graphs = og.get_all_graphs() if hasattr(og, "get_all_graphs") else []
    except Exception:
        graphs = []
    for graph in graphs:
        try:
            for node in graph.get_nodes():
                attr = node.get_attribute("inputs:scriptPath")
                if attr is None or not attr.is_valid():
                    continue
                val = attr.get()
                if val and os.path.abspath(str(val)) == abs_path:
                    force_recompile_scriptnode(graph, node)
                    recompiled.append(node.get_prim_path())
        except Exception:
            continue
    return recompiled


class IsaacAdapterV6(IsaacAdapterBase):
    """Adapter for Isaac Sim 6.0.0 — backend-neutral (PhysX + Newton)."""

    def __init__(self) -> None:
        super().__init__()
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            self._engine = SimulationManager.get_active_physics_engine()
        except Exception:
            self._engine = "unknown"
        try:
            from isaacsim.core.version import get_version

            self._isaacsim_version = str(get_version())
        except Exception:
            self._isaacsim_version = "unknown"
        # Articulation cache keyed by prim_path. Tensor-backed Articulations
        # bind to the current omni.physics.tensors SimulationView; that view
        # is destroyed and recreated on every timeline stop→play cycle, so the
        # cache is cleared on STOP. See _on_timeline_stop.
        self._articulations: Dict[str, Any] = {}
        # Sensor wrappers keyed by prim_path. Replicator annotators fill with
        # data on every render tick — discarding and recreating the wrapper
        # on each capture call (the 5.x pattern) means every call sees a
        # freshly-registered annotator with no accumulated frames, so
        # `get_data()` returns None. Long-lived wrappers let kit's normal
        # update tick populate the annotator between MCP calls.
        self._camera_sensors: Dict[str, Any] = {}
        self._lidar_sensors: Dict[str, Any] = {}
        self._timeline_stop_subscription = None
        try:
            import carb.eventdispatcher
            import omni.timeline

            def _on_timeline_stop(_event):
                self._articulations.clear()
                # Sensor wrappers hold annotator subscriptions; drop them on
                # stop so a fresh play cycle re-registers cleanly.
                self._camera_sensors.clear()
                self._lidar_sensors.clear()

            self._timeline_stop_subscription = (
                carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_STOP,
                    on_event=_on_timeline_stop,
                    observer_name="isaac_sim_mcp.v6.cache_reset_on_stop",
                )
            )
        except Exception:
            pass

    # ── Scene ──────────────────────────────────────────────

    def get_stage(self) -> "Usd.Stage":
        import omni.usd

        return omni.usd.get_context().get_stage()

    def get_assets_root_path(self) -> str:
        from isaacsim.storage.native import get_assets_root_path

        return get_assets_root_path()

    def discover_environments(self) -> Dict[str, Dict[str, str]]:
        # Identical to V5 — uses omni.client, no Isaac Sim physics deps.
        import omni.client
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
        discovered: Dict[str, Dict[str, str]] = {}
        search_bases = ["/Isaac/Environments/", "/NVIDIA/Assets/Scenes/Templates/"]
        for base in search_bases:
            result, entries = omni.client.list(root + base)
            if result != omni.client.Result.OK:
                continue
            for entry in entries:
                name = entry.relative_path.rstrip("/")
                dir_path = root + base + name + "/"
                r2, files = omni.client.list(dir_path)
                if r2 != omni.client.Result.OK:
                    continue
                for f in files:
                    if f.relative_path.endswith(".usd") or f.relative_path.endswith(".usda"):
                        key = name.lower().replace(" ", "_")
                        if key not in discovered:
                            discovered[key] = {
                                "asset_path": base + name + "/" + f.relative_path,
                                "description": name.replace("_", " "),
                            }
                        break
                for f in files:
                    subname = f.relative_path.rstrip("/")
                    r3, subfiles = omni.client.list(dir_path + subname + "/")
                    if r3 != omni.client.Result.OK:
                        continue
                    for sf in subfiles:
                        if sf.relative_path.endswith(".usd") or sf.relative_path.endswith(".usda"):
                            key = f"{name}_{subname}".lower().replace(" ", "_")
                            if key not in discovered:
                                discovered[key] = {
                                    "asset_path": base + name + "/" + subname + "/" + sf.relative_path,
                                    "description": f"{name} {subname}".replace("_", " "),
                                }
                            break
        return discovered

    def load_environment(self, env_path: str, prim_path: str = "/Environment") -> None:
        from isaacsim.core.experimental.utils.stage import add_reference_to_stage

        add_reference_to_stage(env_path, prim_path)

    # ── Prims ──────────────────────────────────────────────

    def create_prim(self, prim_path: str, prim_type: str = "Xform", **kwargs) -> "Usd.Prim":
        from isaacsim.core.experimental.utils.stage import define_prim

        return define_prim(prim_path, type_name=prim_type)

    def delete_prim(self, prim_path: str) -> bool:
        import omni.kit.commands

        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        return True

    def add_reference_to_stage(self, usd_path: str, prim_path: str) -> "Usd.Prim":
        from isaacsim.core.experimental.utils.stage import add_reference_to_stage

        return add_reference_to_stage(usd_path, prim_path)

    def set_prim_transform(
        self,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        rotation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
    ) -> None:
        from pxr import Gf, UsdGeom

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        xformable = UsdGeom.Xformable(prim)
        # Update existing ops in place; only add an op when the caller
        # supplies a value AND the prim doesn't already have that op.
        # ClearXformOpOrder() (the old behaviour) silently reset any axis
        # the caller didn't pass to identity. AddTranslateOp() raises if
        # the op is already in xformOpOrder, so we can't just call it
        # again — look it up first.
        existing = {op.GetName(): op for op in xformable.GetOrderedXformOps()}
        if position is not None:
            op = existing.get("xformOp:translate") or xformable.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            op.Set(Gf.Vec3d(*position))
        if rotation is not None:
            op = existing.get("xformOp:rotateXYZ") or xformable.AddRotateXYZOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            op.Set(Gf.Vec3d(*rotation))
        if scale is not None:
            op = existing.get("xformOp:scale") or xformable.AddScaleOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            op.Set(Gf.Vec3d(*scale))

    def get_prim_transform(self, prim_path: str) -> Dict[str, Any]:
        from pxr import UsdGeom

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        xformable = UsdGeom.Xformable(prim)
        local_transform = xformable.GetLocalTransformation()
        translation = local_transform.ExtractTranslation()
        return {"position": [translation[0], translation[1], translation[2]]}

    def list_prims(self, root_path: str = "/", prim_type: Optional[str] = None) -> List[Dict[str, str]]:
        stage = self.get_stage()
        root = stage.GetPrimAtPath(root_path)
        results: List[Dict[str, str]] = []
        for prim in root.GetAllChildren():
            ptype = prim.GetTypeName()
            if prim_type and ptype != prim_type:
                continue
            results.append({"path": str(prim.GetPath()), "type": ptype})
        return results

    def get_prim_info(self, prim_path: str) -> Dict[str, Any]:
        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        transform = self.get_prim_transform(prim_path)
        children = [str(c.GetPath()) for c in prim.GetAllChildren()]
        info: Dict[str, Any] = {
            "path": prim_path,
            "type": prim.GetTypeName(),
            "transform": transform,
            "children": children,
        }
        if prim.GetTypeName() in ("Cube", "Sphere", "Cylinder", "Cone", "Capsule"):
            try:
                actual_size, _bbox = self.get_prim_actual_size(prim_path)
                info["actual_size"] = actual_size
            except Exception:
                pass
        return info

    def get_prim_actual_size(self, prim_path: str) -> Tuple[List[float], Tuple[List[float], List[float]]]:
        # Identical to V5 — pure pxr/UsdGeom math.
        from pxr import Usd, UsdGeom

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        prim_type = prim.GetTypeName()
        xformable = UsdGeom.Xformable(prim)
        local_transform = xformable.GetLocalTransformation()
        scale = [
            float(local_transform.GetRow3(0).GetLength()),
            float(local_transform.GetRow3(1).GetLength()),
            float(local_transform.GetRow3(2).GetLength()),
        ]
        if prim_type == "Cube":
            geom = UsdGeom.Cube(prim)
            size_attr = geom.GetSizeAttr()
            size = float(size_attr.Get()) if size_attr and size_attr.Get() is not None else 1.0
            dims = [size * scale[0], size * scale[1], size * scale[2]]
        elif prim_type == "Sphere":
            geom = UsdGeom.Sphere(prim)
            radius_attr = geom.GetRadiusAttr()
            radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 0.5
            diameter = radius * 2.0
            dims = [diameter * scale[0], diameter * scale[1], diameter * scale[2]]
        elif prim_type == "Cylinder":
            geom = UsdGeom.Cylinder(prim)
            radius_attr = geom.GetRadiusAttr()
            height_attr = geom.GetHeightAttr()
            axis_attr = geom.GetAxisAttr()
            radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 0.5
            height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 1.0
            axis = axis_attr.Get() if axis_attr and axis_attr.Get() is not None else "Z"
            diameter = radius * 2.0
            if axis == "X":
                dims = [height * scale[0], diameter * scale[1], diameter * scale[2]]
            elif axis == "Y":
                dims = [diameter * scale[0], height * scale[1], diameter * scale[2]]
            else:
                dims = [diameter * scale[0], diameter * scale[1], height * scale[2]]
        elif prim_type == "Cone":
            geom = UsdGeom.Cone(prim)
            radius_attr = geom.GetRadiusAttr()
            height_attr = geom.GetHeightAttr()
            axis_attr = geom.GetAxisAttr()
            radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 0.5
            height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 1.0
            axis = axis_attr.Get() if axis_attr and axis_attr.Get() is not None else "Z"
            diameter = radius * 2.0
            if axis == "X":
                dims = [height * scale[0], diameter * scale[1], diameter * scale[2]]
            elif axis == "Y":
                dims = [diameter * scale[0], height * scale[1], diameter * scale[2]]
            else:
                dims = [diameter * scale[0], diameter * scale[1], height * scale[2]]
        elif prim_type == "Capsule":
            geom = UsdGeom.Capsule(prim)
            radius_attr = geom.GetRadiusAttr()
            height_attr = geom.GetHeightAttr()
            radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 0.5
            height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 1.0
            total_height = height + 2.0 * radius
            diameter = radius * 2.0
            dims = [diameter * scale[0], diameter * scale[1], total_height * scale[2]]
        else:
            raise ValueError(f"Unsupported prim type for size calculation: {prim_type}")
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = world_transform.ExtractTranslation()
        pos = [float(translation[0]), float(translation[1]), float(translation[2])]
        half = [d / 2.0 for d in dims]
        bbox_min = [pos[0] - half[0], pos[1] - half[1], pos[2] - half[2]]
        bbox_max = [pos[0] + half[0], pos[1] + half[1], pos[2] + half[2]]
        return dims, (bbox_min, bbox_max)

    # ── Robots ─────────────────────────────────────────────

    def create_xform_prim(self, prim_path: str) -> Any:
        from isaacsim.core.experimental.prims import XformPrim

        return XformPrim(paths=[prim_path])

    def create_articulation(self, prim_path: str, name: str) -> Any:
        from isaacsim.core.experimental.prims import Articulation

        return Articulation(paths=[prim_path])

    def _new_articulation(self, prim_path: str) -> Any:
        from isaacsim.core.experimental.prims import Articulation

        cached = self._articulations.get(prim_path)
        if cached is not None:
            return cached
        art = Articulation(paths=[prim_path])
        self._articulations[prim_path] = art
        return art

    def discover_robots(self) -> Dict[str, Dict[str, str]]:
        import omni.client
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
        robots_base = root + "/Isaac/Robots/"
        discovered: Dict[str, Dict[str, str]] = {}
        result, manufacturers = omni.client.list(robots_base)
        if result != omni.client.Result.OK:
            return discovered
        for mfr_entry in manufacturers:
            mfr_name = mfr_entry.relative_path.rstrip("/")
            mfr_path = robots_base + mfr_name + "/"
            result2, models = omni.client.list(mfr_path)
            if result2 != omni.client.Result.OK:
                continue
            for model_entry in models:
                model_name = model_entry.relative_path.rstrip("/")
                model_path = mfr_path + model_name + "/"
                result3, files = omni.client.list(model_path)
                if result3 != omni.client.Result.OK:
                    continue
                for file_entry in files:
                    fname = file_entry.relative_path
                    if not (fname.endswith(".usd") or fname.endswith(".usda")):
                        continue
                    asset_rel = f"/Isaac/Robots/{mfr_name}/{model_name}/{fname}"
                    key = model_name.lower().replace(" ", "_")
                    if key in discovered:
                        if len(fname) < len(discovered[key]["asset_path"].split("/")[-1]):
                            discovered[key]["asset_path"] = asset_rel
                    else:
                        discovered[key] = {
                            "asset_path": asset_rel,
                            "description": f"{mfr_name} {model_name}",
                            "manufacturer": mfr_name,
                        }
        return discovered

    def get_robot_joint_info(self, prim_path: str) -> Dict[str, Any]:
        import traceback

        from pxr import Usd, UsdPhysics

        joint_names: List[str] = []
        num_dof = 0
        try:
            self._ensure_physics_world()
            art = self._new_articulation(prim_path)
            joint_names = list(art.dof_names) if art.dof_names else []
            num_dof = int(art.num_dofs) if art.num_dofs else 0
        except Exception as e:
            print(f"v6.get_robot_joint_info: tensor API failed for {prim_path}: {e}")
            traceback.print_exc()

        stage = self.get_stage()
        root_prim = stage.GetPrimAtPath(prim_path)
        if not joint_names and root_prim.IsValid():
            for desc in Usd.PrimRange(root_prim):
                if desc.IsA(UsdPhysics.RevoluteJoint) or desc.IsA(UsdPhysics.PrismaticJoint):
                    joint_names.append(desc.GetName())
            num_dof = len(joint_names)

        joint_limits = []
        for jname in joint_names:
            limit_entry: Dict[str, Any] = {"name": jname}
            for desc in Usd.PrimRange(root_prim):
                if desc.GetName() != jname:
                    continue
                if desc.IsA(UsdPhysics.RevoluteJoint):
                    rev = UsdPhysics.RevoluteJoint(desc)
                    lo = rev.GetLowerLimitAttr().Get()
                    hi = rev.GetUpperLimitAttr().Get()
                    limit_entry["type"] = "revolute"
                    limit_entry["lower"] = float(lo) if lo is not None else None
                    limit_entry["upper"] = float(hi) if hi is not None else None
                    limit_entry["units"] = "degrees"
                    break
                if desc.IsA(UsdPhysics.PrismaticJoint):
                    pris = UsdPhysics.PrismaticJoint(desc)
                    lo = pris.GetLowerLimitAttr().Get()
                    hi = pris.GetUpperLimitAttr().Get()
                    limit_entry["type"] = "prismatic"
                    limit_entry["lower"] = float(lo) if lo is not None else None
                    limit_entry["upper"] = float(hi) if hi is not None else None
                    limit_entry["units"] = "meters"
                    break
            joint_limits.append(limit_entry)
        return {"joint_names": joint_names, "num_dof": num_dof, "joint_limits": joint_limits}

    def set_joint_positions(
        self,
        prim_path: str,
        positions: Sequence[float],
        joint_indices: Optional[List[int]] = None,
    ) -> None:
        import warp as wp

        try:
            self._ensure_physics_world()
            art = self._new_articulation(prim_path)
            positions_arr = wp.array(np.asarray([list(positions)], dtype=np.float32), dtype=wp.float32)
            if joint_indices is not None:
                idx_arr = wp.array(np.asarray(joint_indices, dtype=np.int32), dtype=wp.int32)
                art.set_dof_position_targets(positions_arr, indices=idx_arr)
            else:
                art.set_dof_position_targets(positions_arr)
            return
        except Exception:
            pass
        # USD-drive fallback (sim stopped / articulation not yet initialised)
        self._set_joint_drive_targets(prim_path, positions, joint_indices)

    def _set_joint_drive_targets(
        self,
        prim_path: str,
        positions: Sequence[float],
        joint_indices: Optional[List[int]] = None,
    ) -> None:
        # Identical to V5 — pure pxr.UsdPhysics.
        from pxr import Usd, UsdPhysics

        stage = self.get_stage()
        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        joints = []
        for desc in Usd.PrimRange(root_prim):
            if desc.IsA(UsdPhysics.RevoluteJoint) or desc.IsA(UsdPhysics.PrismaticJoint):
                joints.append(desc)
        if joint_indices is not None:
            targets = list(zip(joint_indices, positions))
        else:
            targets = list(enumerate(positions))
        for idx, value in targets:
            if idx >= len(joints):
                continue
            joint_prim = joints[idx]
            is_revolute = joint_prim.IsA(UsdPhysics.RevoluteJoint)
            drive_type = "angular" if is_revolute else "linear"
            drive = UsdPhysics.DriveAPI.Get(joint_prim, drive_type)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
            if is_revolute:
                drive.GetTargetPositionAttr().Set(float(np.degrees(value)))
            else:
                drive.GetTargetPositionAttr().Set(float(value * 100.0))

    def _get_joint_names(self, prim_path: str) -> List[str]:
        try:
            self._ensure_physics_world()
            art = self._new_articulation(prim_path)
            if art.dof_names:
                return list(art.dof_names)
        except Exception:
            pass
        from pxr import Usd, UsdPhysics

        stage = self.get_stage()
        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            return []
        names: List[str] = []
        for desc in Usd.PrimRange(root_prim):
            if desc.IsA(UsdPhysics.RevoluteJoint) or desc.IsA(UsdPhysics.PrismaticJoint):
                names.append(desc.GetName())
        return names

    def get_joint_positions(self, prim_path: str) -> List[float]:
        try:
            self._ensure_physics_world()
            art = self._new_articulation(prim_path)
            positions = art.get_dof_positions()
            if positions is not None:
                # batched (1, num_dofs) wp.array → flat list
                arr = positions.numpy() if hasattr(positions, "numpy") else np.asarray(positions)
                return arr.reshape(-1).tolist()
        except Exception:
            pass
        # USD fallback identical to V5
        from pxr import Usd, UsdPhysics

        stage = self.get_stage()
        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            return []
        positions_list: List[float] = []
        for desc in Usd.PrimRange(root_prim):
            if not (desc.IsA(UsdPhysics.RevoluteJoint) or desc.IsA(UsdPhysics.PrismaticJoint)):
                continue
            is_revolute = desc.IsA(UsdPhysics.RevoluteJoint)
            drive_type = "angular" if is_revolute else "linear"
            drive = UsdPhysics.DriveAPI.Get(desc, drive_type)
            if drive:
                target = drive.GetTargetPositionAttr().Get()
                if target is not None:
                    if is_revolute:
                        positions_list.append(float(np.radians(target)))
                    else:
                        positions_list.append(float(target / 100.0))
                else:
                    positions_list.append(0.0)
            else:
                positions_list.append(0.0)
        return positions_list

    def get_joint_config(self, prim_path: str) -> Dict[str, Any]:
        from pxr import Usd, UsdPhysics

        self._ensure_physics_world()
        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        joint_names = self._get_joint_names(prim_path)
        current_pos_list = self.get_joint_positions(prim_path)

        runtime_targets: List[float] = []
        try:
            art = self._new_articulation(prim_path)
            targets = art.get_dof_position_targets()
            if targets is not None:
                arr = targets.numpy() if hasattr(targets, "numpy") else np.asarray(targets)
                runtime_targets = arr.reshape(-1).tolist()
        except Exception:
            pass

        joints_info = []
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdPhysics.RevoluteJoint) or desc.IsA(UsdPhysics.PrismaticJoint):
                joint_data: Dict[str, Any] = {"name": desc.GetName()}
                if desc.IsA(UsdPhysics.RevoluteJoint):
                    joint_data["type"] = "revolute"
                    joint_api = UsdPhysics.RevoluteJoint(desc)
                else:
                    joint_data["type"] = "prismatic"
                    joint_api = UsdPhysics.PrismaticJoint(desc)
                lower_attr = joint_api.GetLowerLimitAttr()
                upper_attr = joint_api.GetUpperLimitAttr()
                joint_data["lower_limit"] = lower_attr.Get() if lower_attr else None
                joint_data["upper_limit"] = upper_attr.Get() if upper_attr else None
                for drive_type in ["angular", "linear"]:
                    drive_api = UsdPhysics.DriveAPI.Get(desc, drive_type)
                    if drive_api:
                        joint_data["drive_type"] = drive_type
                        stiffness_attr = drive_api.GetStiffnessAttr()
                        damping_attr = drive_api.GetDampingAttr()
                        target_attr = drive_api.GetTargetPositionAttr()
                        joint_data["stiffness"] = stiffness_attr.Get() if stiffness_attr else None
                        joint_data["damping"] = damping_attr.Get() if damping_attr else None
                        joint_data["target_position"] = target_attr.Get() if target_attr else None
                        break
                jname = desc.GetName()
                if jname in joint_names:
                    idx = joint_names.index(jname)
                    if idx < len(current_pos_list):
                        joint_data["actual_position"] = current_pos_list[idx]
                    if idx < len(runtime_targets):
                        joint_data["target_position"] = float(runtime_targets[idx])
                    if joint_data.get("target_position") is not None and "actual_position" in joint_data:
                        joint_data["position_error"] = joint_data["target_position"] - joint_data["actual_position"]
                joints_info.append(joint_data)

        warnings = []
        for j in joints_info:
            stiff = j.get("stiffness")
            damp = j.get("damping")
            if stiff is not None and stiff == 0 and (damp is None or damp == 0):
                warnings.append(
                    f"Joint '{j['name']}' has stiffness=0 and damping=0 — "
                    f"its drive is effectively disabled and will not respond to position targets."
                )
        result: Dict[str, Any] = {
            "prim_path": prim_path,
            "joint_count": len(joints_info),
            "joints": joints_info,
        }
        if warnings:
            result["warnings"] = warnings
        return result

    # ── Physics ────────────────────────────────────────────

    def _ensure_physics_world(self) -> None:
        """Initialise SimulationManager (idempotent under both PhysX and Newton).

        Cleans stale PhysicsScene references first — the SimulationManager
        retains Python wrappers around scenes that may have been deleted via
        clear_scene, and calling setup_simulation/initialize_physics against
        them raises "Accessed invalid expired 'PhysicsScene' prim".
        """
        from isaacsim.core.simulation_manager import SimulationManager

        try:
            SimulationManager._cleanup_stale_physics_scenes()
        except Exception:
            pass
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        SimulationManager.initialize_physics()

    def create_world(self, **kwargs) -> Any:
        """V6 exposes SimulationManager (a class-level singleton) where V5 returned World()."""
        from isaacsim.core.simulation_manager import SimulationManager

        return SimulationManager

    def create_simulation_context(self, **kwargs) -> Any:
        from isaacsim.core.simulation_manager import SimulationManager

        return SimulationManager

    def create_physics_scene(self, gravity: Optional[Sequence[float]] = None, scene_name: str = "PhysicsScene") -> str:
        import omni.kit.commands

        scene_path = f"/World/{scene_name}"
        omni.kit.commands.execute("CreatePrim", prim_path=scene_path, prim_type="PhysicsScene")
        return scene_path

    def get_physics_state(self, prim_path: str) -> Dict[str, Any]:
        from pxr import UsdPhysics

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")

        result: Dict[str, Any] = {"prim_path": prim_path}
        has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        result["has_rigid_body"] = has_rb
        if has_rb:
            rb = UsdPhysics.RigidBodyAPI(prim)
            kinematic_attr = rb.GetKinematicEnabledAttr()
            result["is_kinematic"] = kinematic_attr.Get() if kinematic_attr else False
        has_mass = prim.HasAPI(UsdPhysics.MassAPI)
        if has_mass:
            mass_api = UsdPhysics.MassAPI(prim)
            mass_attr = mass_api.GetMassAttr()
            result["mass"] = mass_attr.Get() if mass_attr else None
        result["collision_enabled"] = prim.HasAPI(UsdPhysics.CollisionAPI)

        if has_rb:
            lin_vel = [0.0, 0.0, 0.0]
            ang_vel = [0.0, 0.0, 0.0]
            try:
                from isaacsim.core.simulation_manager import SimulationManager

                view = SimulationManager.get_physics_simulation_view()
                if view is not None:
                    rb_view = view.create_rigid_body_view([prim_path])
                    vels = rb_view.get_velocities()
                    arr = vels.numpy() if hasattr(vels, "numpy") else np.asarray(vels)
                    if arr.size >= 6:
                        flat = arr.reshape(-1)[:6]
                        lin_vel = [float(flat[0]), float(flat[1]), float(flat[2])]
                        ang_vel = [float(flat[3]), float(flat[4]), float(flat[5])]
            except Exception:
                pass
            result["linear_velocity"] = lin_vel
            result["angular_velocity"] = ang_vel

        result["contacts"] = []
        return result

    # ── Sensors ────────────────────────────────────────────

    def create_camera(self, prim_path: str, resolution: Tuple[int, int] = (1280, 720), **kwargs) -> Any:
        # 6.0 RtxCamera takes a single `path: str` — the 5.x batched
        # (`prim_paths=[...], resolutions=[...]`) signature was removed.
        # Also stand up the CameraSensor runtime + RGB annotator now so kit's
        # background render ticks start filling the annotator immediately;
        # later capture_image calls read accumulated frames from the cache.
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        camera = RtxCamera(path=prim_path)
        # CameraSensor expects (height, width). Adapter callers historically
        # pass (width, height) — translate so the cached resolution is sane.
        h, w = (resolution[1], resolution[0]) if len(resolution) == 2 else (720, 1280)
        self._camera_sensors[prim_path] = CameraSensor(
            path=prim_path, resolution=(h, w), annotators=["rgb"]
        )
        return camera

    def capture_camera_image(self, prim_path: str) -> np.ndarray:
        # Reuse the wrapper cached by create_camera. Building a fresh
        # CameraSensor on every call re-registers the annotator with the
        # render pipeline and discards any frames produced since the prim
        # was created, so `get_data` returns None — that was the root cause
        # of the "empty data" symptom. With a long-lived wrapper, kit's
        # background update tick fills the annotator between MCP commands
        # and get_data returns the latest rendered frame.
        from isaacsim.sensors.experimental.rtx import CameraSensor

        sensor = self._camera_sensors.get(prim_path)
        if sensor is None:
            sensor = CameraSensor(path=prim_path, resolution=(720, 1280), annotators=["rgb"])
            self._camera_sensors[prim_path] = sensor
        data, _info = sensor.get_data("rgb")
        if data is None:
            return np.zeros((0,), dtype=np.uint8)
        return data.numpy() if hasattr(data, "numpy") else np.asarray(data)

    def create_lidar(self, prim_path: str, config: Optional[str] = None, **kwargs) -> Any:
        # 6.0 Lidar takes a single `path: str`. Hardware preset (formerly the
        # `config` arg) is now set through schema attributes after creation;
        # the bare constructor produces a generic OmniLidar prim. As with
        # create_camera, also cache a LidarSensor wrapper so its annotator
        # starts producing data on kit's regular render tick.
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

        lidar = Lidar(path=prim_path)
        self._lidar_sensors[prim_path] = LidarSensor(
            path=prim_path, annotators=["generic-model-output"]
        )
        return lidar

    def get_lidar_point_cloud(self, prim_path: str) -> np.ndarray:
        # 6.0 LidarSensor uses the unified "generic-model-output" annotator;
        # the 5.x `RtxSensorCpu+IsaacComputeRTXLidarPointCloud` chain is gone.
        # See `capture_camera_image` for the caching rationale.
        from isaacsim.sensors.experimental.rtx import LidarSensor

        sensor = self._lidar_sensors.get(prim_path)
        if sensor is None:
            sensor = LidarSensor(path=prim_path, annotators=["generic-model-output"])
            self._lidar_sensors[prim_path] = sensor
        data, info = sensor.get_data("generic-model-output")
        if data is None:
            return np.zeros((0, 3), dtype=np.float32)
        return data.numpy() if hasattr(data, "numpy") else np.asarray(data)

    # ── Materials ──────────────────────────────────────────

    def create_pbr_material(
        self,
        prim_path: str,
        color: Optional[Sequence[float]] = None,
        roughness: float = 0.5,
        metallic: float = 0.0,
    ) -> Any:
        from pxr import Gf, Sdf, UsdShade

        stage = self.get_stage()
        material = UsdShade.Material.Define(stage, prim_path)
        shader = UsdShade.Shader.Define(stage, f"{prim_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        if color:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color[:3]))
        material.CreateSurfaceOutput().ConnectToSource(shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        return material

    def create_physics_material(
        self,
        prim_path: str,
        static_friction: float = 0.5,
        dynamic_friction: float = 0.5,
        restitution: float = 0.0,
    ) -> Any:
        from pxr import UsdPhysics

        stage = self.get_stage()
        material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(prim_path))
        material.CreateStaticFrictionAttr(static_friction)
        material.CreateDynamicFrictionAttr(dynamic_friction)
        material.CreateRestitutionAttr(restitution)
        return material

    def apply_material(self, material_path: str, target_prim_path: str) -> None:
        from pxr import UsdShade

        stage = self.get_stage()
        material = UsdShade.Material(stage.GetPrimAtPath(material_path))
        target = stage.GetPrimAtPath(target_prim_path)
        UsdShade.MaterialBindingAPI(target).Bind(material)

    # ── Lighting ───────────────────────────────────────────

    def create_light(
        self,
        light_type: str,
        prim_path: str,
        intensity: float = 1000.0,
        color: Optional[Sequence[float]] = None,
        **kwargs,
    ) -> Any:
        from pxr import Gf, UsdLux

        stage = self.get_stage()
        light_classes = {
            "DistantLight": UsdLux.DistantLight,
            "DomeLight": UsdLux.DomeLight,
            "SphereLight": UsdLux.SphereLight,
            "RectLight": UsdLux.RectLight,
            "DiskLight": UsdLux.DiskLight,
            "CylinderLight": UsdLux.CylinderLight,
        }
        cls = light_classes.get(light_type)
        if not cls:
            raise ValueError(f"Unknown light type: {light_type}. Options: {list(light_classes.keys())}")
        light = cls.Define(stage, prim_path)
        light.CreateIntensityAttr(intensity)
        if color:
            light.CreateColorAttr(Gf.Vec3f(*color[:3]))
        position = kwargs.get("position")
        if position:
            self.set_prim_transform(prim_path, position=position)
        rotation = kwargs.get("rotation")
        if rotation:
            self.set_prim_transform(prim_path, rotation=rotation)
        return light

    def modify_light(
        self,
        prim_path: str,
        intensity: Optional[float] = None,
        color: Optional[Sequence[float]] = None,
    ) -> None:
        from pxr import Gf

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Light not found: {prim_path}")
        if intensity is not None:
            prim.GetAttribute("inputs:intensity").Set(intensity)
        if color is not None:
            prim.GetAttribute("inputs:color").Set(Gf.Vec3f(*color[:3]))

    # ── Assets ─────────────────────────────────────────────

    def clone_prim(self, source_path: str, target_path: str) -> None:
        import omni.kit.commands

        omni.kit.commands.execute("CopyPrim", path_from=source_path, path_to=target_path)

    # ── Assets ─────────────────────────────────────────────

    def import_urdf(self, urdf_path: str, prim_path: str = "/World/robot", **kwargs) -> Any:
        # 6.0 splits URDF import into two steps:
        #   1) URDFImporter.import_urdf() converts the .urdf to a .usd on disk
        #      (the `dest_path` kwarg from 5.x is gone — output dir is chosen
        #      via `usd_path` on the config, defaulting to the URDF's directory)
        #   2) the caller references that .usd into the live stage
        import os
        import tempfile

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        usd_out_dir = kwargs.pop("usd_path", None) or tempfile.mkdtemp(prefix="urdf_import_")
        config = URDFImporterConfig(urdf_path=urdf_path, usd_path=usd_out_dir, **kwargs)
        importer = URDFImporter(config)
        usd_path = importer.import_urdf()
        # Bring the generated USD into the live stage at the requested prim path
        return self.add_reference_to_stage(usd_path, prim_path)

    # ── Simulation ─────────────────────────────────────────

    def play(self) -> None:
        import omni.timeline

        self._ensure_physics_world()
        omni.timeline.get_timeline_interface().play()

    def pause(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().pause()

    def stop(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
        # Restore articulations / rigid bodies to their spawn pose (the state
        # captured at first Play), matching the Isaac UI Stop button. Guarded so
        # a scene with no initialised physics still stops cleanly.
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            SimulationManager.reset_simulation()
        except Exception:
            pass

    def step(
        self,
        num_steps: int = 1,
        observe_prims: Optional[List[str]] = None,
        observe_joints: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        # SimulationManager.step pumps only the physics pipeline (no asyncio
        # event-loop reentry), which avoids the "Cannot enter into task" errors
        # that omni.kit.app.update() triggers when called from inside the MCP
        # dispatch coroutine on Kit 107 (Isaac Sim 6.0).
        from isaacsim.core.simulation_manager import SimulationManager

        self._ensure_physics_world()
        SimulationManager.step(steps=num_steps)

        result: Dict[str, Any] = {"stepped": num_steps}

        if observe_prims:
            from pxr import UsdPhysics

            prim_states = []
            stage = self.get_stage()
            for path in observe_prims:
                prim = stage.GetPrimAtPath(path)
                if not prim.IsValid():
                    prim_states.append({"prim_path": path, "error": "Prim not found"})
                    continue
                state: Dict[str, Any] = {"prim_path": path}
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    try:
                        from isaacsim.core.simulation_manager import SimulationManager

                        view = SimulationManager.get_physics_simulation_view()
                        rb_view = view.create_rigid_body_view([path]) if view is not None else None
                        if rb_view is not None:
                            transforms = rb_view.get_transforms()
                            arr = transforms.numpy() if hasattr(transforms, "numpy") else np.asarray(transforms)
                            if arr.size >= 3:
                                flat = arr.reshape(-1)
                                state["position"] = [float(flat[0]), float(flat[1]), float(flat[2])]
                        else:
                            transform = self.get_prim_transform(path)
                            state["position"] = transform.get("position", [0, 0, 0])
                    except Exception:
                        transform = self.get_prim_transform(path)
                        state["position"] = transform.get("position", [0, 0, 0])
                else:
                    transform = self.get_prim_transform(path)
                    state["position"] = transform.get("position", [0, 0, 0])
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    try:
                        ps = self.get_physics_state(path)
                        state["linear_velocity"] = ps.get("linear_velocity", [0, 0, 0])
                        state["angular_velocity"] = ps.get("angular_velocity", [0, 0, 0])
                    except Exception:
                        pass
                prim_states.append(state)
            result["prim_states"] = prim_states

        if observe_joints:
            joint_states = []
            for path in observe_joints:
                try:
                    positions = self.get_joint_positions(path)
                    names = self._get_joint_names(path)
                    joints_dict = dict(zip(names, positions)) if names else {"positions": positions}
                    joint_states.append({"prim_path": path, "joints": joints_dict})
                except Exception as e:
                    joint_states.append({"prim_path": path, "error": str(e)})
            result["joint_states"] = joint_states

        return result

    def get_simulation_state(self) -> Dict[str, Any]:
        import omni.timeline
        from pxr import UsdPhysics

        timeline = omni.timeline.get_timeline_interface()
        is_playing = timeline.is_playing()
        is_stopped = timeline.is_stopped()
        if is_playing:
            timeline_state = "playing"
        elif is_stopped:
            timeline_state = "stopped"
        else:
            timeline_state = "paused"

        current_time = timeline.get_current_time()
        stage = self.get_stage()
        physics_dt = 1.0 / 60.0
        for prim in stage.Traverse():
            try:
                if prim.HasAPI(UsdPhysics.Scene):
                    time_step_attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
                    if time_step_attr and time_step_attr.Get():
                        steps_per_sec = time_step_attr.Get()
                        if steps_per_sec > 0:
                            physics_dt = 1.0 / steps_per_sec
                    break
            except Exception:
                pass

        return {
            "timeline_state": timeline_state,
            "current_time": current_time,
            "physics_dt": physics_dt,
            "engine": self._engine,
            "isaacsim_version": self._isaacsim_version,
        }

    def execute_script(self, code: str, cwd: Optional[str] = None) -> Dict[str, Any]:
        import io
        import sys
        import traceback

        import carb
        import omni
        from pxr import Gf, Sdf, Usd, UsdGeom

        if cwd and cwd not in sys.path:
            sys.path.insert(0, cwd)

        local_ns = {"omni": omni, "carb": carb, "Usd": Usd, "UsdGeom": UsdGeom, "Sdf": Sdf, "Gf": Gf}

        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout = captured_out = io.StringIO()
        sys.stderr = captured_err = io.StringIO()
        try:
            self._ensure_physics_world()
            exec(code, local_ns)
            return {
                "status": "success",
                "message": "Script executed successfully",
                "stdout": captured_out.getvalue(),
                "stderr": captured_err.getvalue(),
            }
        except Exception as e:
            return {
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc(),
                "stdout": captured_out.getvalue(),
                "stderr": captured_err.getvalue(),
            }
        finally:
            sys.stdout, sys.stderr = old_stdout, old_stderr

    _exec_namespaces: Dict[str, dict] = {}

    def reload_script(self, file_path: str, module_name: Optional[str] = None) -> Dict[str, Any]:
        import importlib
        import io
        import os
        import sys
        import traceback

        parent_dir = os.path.dirname(os.path.abspath(file_path))
        if parent_dir not in sys.path:
            sys.path.insert(0, parent_dir)

        abs_path = os.path.abspath(file_path)

        # ScriptNode-aware reload: if any Action-Graph ScriptNode references this
        # file via inputs:scriptPath, force it to recompile (the standalone
        # re-exec below would not touch the running graph node).
        recompiled = _recompile_scriptnodes_for_file(abs_path)
        if recompiled:
            return {
                "status": "success",
                "message": f"Recompiled ScriptNode(s) referencing {os.path.basename(file_path)}",
                "recompiled_nodes": recompiled,
            }

        old_ns = self._exec_namespaces.get(abs_path)
        if old_ns:
            for key, val in old_ns.items():
                if hasattr(val, "unsubscribe"):
                    try:
                        val.unsubscribe()
                    except Exception:
                        pass

        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout = captured_out = io.StringIO()
        sys.stderr = captured_err = io.StringIO()
        try:
            if module_name:
                if module_name in sys.modules:
                    _module = importlib.reload(sys.modules[module_name])
                    msg = f"Module '{module_name}' reloaded successfully"
                else:
                    _module = importlib.import_module(module_name)
                    msg = f"Module '{module_name}' imported successfully"
            else:
                if not os.path.isfile(file_path):
                    return {"status": "error", "message": f"File not found: {file_path}"}
                with open(file_path, "r") as f:
                    code = f.read()
                import carb
                import omni
                from pxr import Gf, Sdf, Usd, UsdGeom

                local_ns = {
                    "omni": omni,
                    "carb": carb,
                    "Usd": Usd,
                    "UsdGeom": UsdGeom,
                    "Sdf": Sdf,
                    "Gf": Gf,
                    "__file__": file_path,
                }
                self._ensure_physics_world()
                exec(code, local_ns)
                self._exec_namespaces[abs_path] = local_ns
                msg = f"Script '{os.path.basename(file_path)}' executed successfully"

            return {
                "status": "success",
                "message": msg,
                "stdout": captured_out.getvalue(),
                "stderr": captured_err.getvalue(),
            }
        except Exception as e:
            return {
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc(),
                "stdout": captured_out.getvalue(),
                "stderr": captured_err.getvalue(),
            }
        finally:
            sys.stdout, sys.stderr = old_stdout, old_stderr
