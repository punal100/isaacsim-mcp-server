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

"""Sensor creation and data capture command handlers."""

from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

from ..adapters.base import IsaacAdapterBase


def register(registry: Dict[str, Any], adapter: IsaacAdapterBase) -> None:
    registry["sensors.create_camera"] = lambda **p: create_camera(adapter, **p)
    registry["sensors.capture_image"] = lambda **p: capture_image(adapter, **p)
    registry["sensors.create_lidar"] = lambda **p: create_lidar(adapter, **p)
    registry["sensors.get_point_cloud"] = lambda **p: get_point_cloud(adapter, **p)


def create_camera(
    adapter: IsaacAdapterBase,
    prim_path: str = "/World/Camera",
    position: Optional[Sequence[float]] = None,
    rotation: Optional[Sequence[float]] = None,
    resolution: Optional[Sequence[int]] = None,
) -> Dict[str, Any]:
    try:
        res = tuple(resolution) if resolution else (1280, 720)
        _cam = adapter.create_camera(prim_path, resolution=res)
        if position or rotation:
            adapter.set_prim_transform(prim_path, position=position, rotation=rotation)
        return {"status": "success", "message": f"Camera created at {prim_path}", "prim_path": prim_path}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def capture_image(
    adapter: IsaacAdapterBase, prim_path: str = "/World/Camera", output_path: Optional[str] = None
) -> Dict[str, Any]:
    try:
        image_data = adapter.capture_camera_image(prim_path)
        # An RTX sensor with no frame yet yields an empty array, not an error.
        # Reporting that as success gave back {"shape": [0]} with status
        # "success", which a caller cannot tell apart from a captured image —
        # and with output_path set it fed an empty array to Image.fromarray.
        # Verified on Isaac Sim 6.0.1: in the step-only debug loop the timeline
        # never plays, Replicator's orchestrator therefore stays STOPPED
        # (/omni/replicator/captureOnPlay defaults to True), and every capture
        # returned an empty array while reporting success.
        if image_data is None or getattr(image_data, "size", 0) == 0:
            # Only say a render was requested if this adapter can actually
            # request one. V6 schedules a Replicator frame; V5 has no such path,
            # and telling a 5.1 caller to "call again to collect it" would send
            # them round a loop that never terminates.
            # Test the capability, not the current value: _render_request starts
            # as None, so checking it would give a V6 caller the V5 wording on
            # the first call — the one that actually schedules the render.
            requested = callable(getattr(adapter, "_request_render_frame", None))
            remedy = (
                "A render has been requested — call capture_image again to collect it."
                if requested
                else "Play the simulation, or capture again once a frame has rendered."
            )
            return {
                "status": "error",
                "message": (
                    f"No frame available from {prim_path} yet. RTX sensor data is produced by "
                    "Replicator, which by default only captures while the timeline is playing "
                    f"(/omni/replicator/captureOnPlay). {remedy}"
                ),
            }
        if output_path:
            from PIL import Image

            img = Image.fromarray(image_data)
            img.save(output_path)
            return {"status": "success", "message": f"Image saved to {output_path}", "output_path": output_path}
        return {
            "status": "success",
            "message": "Image captured",
            "shape": list(image_data.shape) if hasattr(image_data, "shape") else None,
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


def create_lidar(
    adapter: IsaacAdapterBase,
    prim_path: str = "/World/Lidar",
    position: Optional[Sequence[float]] = None,
    rotation: Optional[Sequence[float]] = None,
    config: Optional[str] = None,
) -> Dict[str, Any]:
    try:
        adapter.create_lidar(prim_path, config=config)
        if position or rotation:
            adapter.set_prim_transform(prim_path, position=position, rotation=rotation)
        return {"status": "success", "message": f"Lidar created at {prim_path}", "prim_path": prim_path}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def get_point_cloud(
    adapter: IsaacAdapterBase,
    prim_path: str = "/World/Lidar",
    max_points: Optional[int] = None,
    output_path: Optional[str] = None,
) -> Dict[str, Any]:
    try:
        pc = adapter.get_lidar_point_cloud(prim_path)
        point_count = len(pc) if pc is not None else 0
        # An empty read has three different causes and they need different
        # answers. The message used to give one -- "call play_simulation" --
        # which is wrong advice two times out of three: it is baffling when you
        # are already playing, and actively misleading when the lidar is simply
        # looking at nothing (a sensor buried inside a robot returned 491 points
        # in testing, and would return 0 if fully enclosed).
        if point_count == 0:
            timeline_state = ""
            try:
                timeline_state = str((adapter.get_simulation_state() or {}).get("timeline_state", "")).lower()
            except Exception:
                pass

            if not timeline_state:
                # Could not tell. Cover both, rather than guessing and sending
                # the caller down the wrong path.
                message = (
                    f"No lidar data from {prim_path}. If the timeline is not running, call "
                    "play_simulation — RTX lidar is produced by Replicator only while the sim runs. "
                    "If it is already playing, retry: the sensor fills only on frames where a "
                    "rotation completes. If it never fills, check the lidar is not inside geometry."
                )
            elif timeline_state != "playing":
                message = (
                    f"No lidar data from {prim_path}: the timeline is {timeline_state}. RTX lidar is "
                    "produced by Replicator only while the sim runs, and one rendered frame is not "
                    "enough — call play_simulation, then read again."
                )
            else:
                # Playing, so frames are flowing. This annotator only yields on
                # frames where a sweep completes, so an empty read is usually
                # "not this frame" and a retry fixes it.
                message = (
                    f"No completed sweep from {prim_path} on this frame. The sensor fills only on "
                    "frames where a rotation completes, so retry the same call — several attempts "
                    "over a few seconds is normal. If it never fills, check the lidar is not inside "
                    "geometry: one placed at a robot's own origin sees only the robot."
                )

            return {
                "status": "error",
                "message": message,
                "point_count": 0,
                "timeline_state": timeline_state or "unknown",
            }
        # The decoded cloud used to be dropped here and only its length
        # returned, so a tool named get_lidar_point_cloud could not produce a
        # point cloud. Returning all of it is not the answer either: a sweep is
        # tens of thousands of points and megabytes of JSON, which is ruinous
        # for an agent's context. So the default is decision-grade summary, with
        # the points available on request and the full array writable to disk --
        # the same escape hatch capture_image offers via output_path.
        #
        # Deliberately plain Python rather than numpy: the unit suite stubs
        # numpy, and a summary that only works inside Kit is a summary nobody
        # tests.
        rows = [(float(p[0]), float(p[1]), float(p[2])) for p in pc]
        min_x = min_y = min_z = float("inf")
        max_x = max_y = max_z = float("-inf")
        nearest_sq = float("inf")
        nearest_point = rows[0]
        for x, y, z in rows:
            if x < min_x:
                min_x = x
            if y < min_y:
                min_y = y
            if z < min_z:
                min_z = z
            if x > max_x:
                max_x = x
            if y > max_y:
                max_y = y
            if z > max_z:
                max_z = z
            d2 = x * x + y * y + z * z
            if d2 < nearest_sq:
                nearest_sq, nearest_point = d2, (x, y, z)

        result: Dict[str, Any] = {
            "status": "success",
            "message": f"Got {point_count} points",
            "point_count": point_count,
            "bounds": {
                "min": [round(min_x, 4), round(min_y, 4), round(min_z, 4)],
                "max": [round(max_x, 4), round(max_y, 4), round(max_z, 4)],
            },
            "nearest": {
                "distance": round(nearest_sq**0.5, 4),
                "point": [round(v, 4) for v in nearest_point],
            },
            "frame": "sensor-local coordinates, meters",
        }

        if output_path:
            # .npy so the caller gets every point at full precision;
            # numpy.load(path) reads it back as an (N, 3) array.
            try:
                import numpy as np

                target = output_path if str(output_path).endswith(".npy") else f"{output_path}.npy"
                np.save(target, np.asarray(rows, dtype="float32"))
                result["output_path"] = target
            except Exception as exc:
                result["output_error"] = f"could not write {output_path}: {exc}"

        if max_points:
            limit = max(1, int(max_points))
            if point_count > limit:
                # Even stride rather than the first N: a sweep is ordered by
                # beam, so the head of the array is one slice of the scene.
                stride = -(-point_count // limit)
                sample = rows[::stride][:limit]
                result["sampled"] = True
                result["sample_stride"] = stride
            else:
                sample = rows
                result["sampled"] = False
            result["points"] = [[round(v, 4) for v in row] for row in sample]

        return result
    except Exception as e:
        return {"status": "error", "message": str(e)}
