#!/usr/bin/env python3
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
"""Live smoke test against a running Isaac Sim 6.0 + MCP extension.

Connects to the extension socket (default localhost:8766), sends one
command per V6 surface area, and prints pass/fail per check.

Prerequisites:
    1. Isaac Sim 6.0 running with the MCP extension enabled
       (either via isaac-sim.sh or isaac-sim.newton.sh).
    2. The extension has been hot-reloaded with the V6 code on disk
       (run scripts/dev_mcp_server.sh once, then this script).

Usage:
    python scripts/smoke_test_v6.py
    python scripts/smoke_test_v6.py --port 8767
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any, Dict


def send(host: str, port: int, cmd_type: str, params: Dict[str, Any]) -> Dict[str, Any]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30)
    sock.connect((host, port))
    try:
        payload = json.dumps({"type": cmd_type, "params": params}).encode("utf-8")
        sock.sendall(payload)
        chunks = []
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            try:
                return json.loads(b"".join(chunks).decode("utf-8"))
            except json.JSONDecodeError:
                continue
        return {"status": "error", "message": "Connection closed without complete JSON"}
    finally:
        sock.close()


def check(name: str, response: Dict[str, Any], assertion=None) -> bool:
    status = response.get("status")
    if status != "success":
        print(f"  [FAIL] {name}: {response.get('message')}", file=sys.stderr)
        return False
    if assertion is not None:
        ok, why = assertion(response.get("result", {}))
        if not ok:
            print(f"  [FAIL] {name}: {why}", file=sys.stderr)
            return False
    print(f"  [ OK ] {name}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()

    print(f"Connecting to Isaac Sim MCP extension at {args.host}:{args.port}...")

    results = []

    # 1. Engine + version field exposed by V6
    resp = send(args.host, args.port, "simulation.get_state", {})
    def _check_state(r: Dict[str, Any]) -> tuple[bool, str]:
        if "engine" not in r:
            return False, "missing 'engine' field — V5 adapter is in use, not V6"
        if r["engine"] not in ("physx", "newton", "remotesim"):
            return False, f"unexpected engine: {r.get('engine')}"
        if "isaacsim_version" not in r or not r["isaacsim_version"].startswith("6."):
            return False, f"unexpected isaacsim_version: {r.get('isaacsim_version')}"
        return True, ""
    results.append(check("simulation.get_state shows engine + isaacsim_version", resp, _check_state))
    engine = resp.get("result", {}).get("engine", "unknown")
    print(f"        engine={engine}, version={resp.get('result', {}).get('isaacsim_version')}")

    # 2. Scene info round-trip
    resp = send(args.host, args.port, "scene.get_info", {})
    results.append(check("scene.get_info", resp))

    # 3. Create a cube and verify info
    send(args.host, args.port, "scene.clear", {})
    resp = send(args.host, args.port, "objects.create", {
        "object_type": "cube",
        "prim_path": "/World/SmokeCube",
        "size": 0.5,
        "position": [0.0, 0.0, 1.0],
    })
    results.append(check("objects.create cube", resp))

    resp = send(args.host, args.port, "scene.get_prim_info", {"prim_path": "/World/SmokeCube"})
    def _is_cube(r: Dict[str, Any]) -> tuple[bool, str]:
        if r.get("type") != "Cube":
            return False, f"expected type=Cube, got {r.get('type')}"
        return True, ""
    results.append(check("scene.get_prim_info on cube", resp, _is_cube))

    # 4. Create a physics scene + ground plane, play, step a few frames
    resp = send(args.host, args.port, "simulation.set_physics", {"gravity": [0.0, 0.0, -9.81]})
    results.append(check("simulation.set_physics (creates PhysicsScene)", resp))

    resp = send(args.host, args.port, "simulation.play", {})
    results.append(check("simulation.play", resp))

    resp = send(args.host, args.port, "simulation.step", {
        "num_steps": 30,
        "observe_prims": ["/World/SmokeCube"],
    })
    def _stepped(r: Dict[str, Any]) -> tuple[bool, str]:
        if "prim_states" not in r:
            return False, "no prim_states in step response"
        states = r["prim_states"]
        if not states or "position" not in states[0]:
            return False, "no position observed for cube"
        return True, ""
    results.append(check("simulation.step with observe_prims (physics view read)", resp, _stepped))

    resp = send(args.host, args.port, "simulation.stop", {})
    results.append(check("simulation.stop", resp))

    # 4b. stop_simulation must reset the scene to spawn state: create a cube
    #     above the ground, play, step until it falls, stop, and verify the
    #     cube's world Z is back at its spawn value (not the fallen value).
    send(args.host, args.port, "scene.clear", {})
    resp = send(args.host, args.port, "simulation.set_physics", {"gravity": [0.0, 0.0, -9.81]})
    results.append(check("simulation.set_physics (reset test)", resp))

    spawn_z = 2.0
    resp = send(args.host, args.port, "objects.create", {
        "object_type": "cube",
        "prim_path": "/World/ResetCube",
        "size": 0.5,
        "position": [0.0, 0.0, spawn_z],
    })
    results.append(check("objects.create cube above ground (reset test)", resp))

    resp = send(args.host, args.port, "simulation.play", {})
    results.append(check("simulation.play (reset test)", resp))

    resp = send(args.host, args.port, "simulation.step", {
        "num_steps": 60,
        "observe_prims": ["/World/ResetCube"],
    })
    def _fell(r: Dict[str, Any]) -> tuple[bool, str]:
        states = r.get("prim_states") or []
        if not states or "position" not in states[0]:
            return False, "no position observed for cube"
        z = states[0]["position"][2]
        if z >= spawn_z:
            return False, f"cube did not fall: z={z}"
        return True, ""
    results.append(check("simulation.step lets cube fall (reset test)", resp, _fell))

    resp = send(args.host, args.port, "simulation.stop", {})
    results.append(check("simulation.stop (reset test)", resp))

    resp = send(args.host, args.port, "scene.get_prim_info", {"prim_path": "/World/ResetCube"})
    def _back_at_spawn(r: Dict[str, Any]) -> tuple[bool, str]:
        position = r.get("position")
        if not position:
            return False, "no position in prim_info"
        z = position[2]
        if abs(z - spawn_z) > 1e-3:
            return False, f"expected z~={spawn_z} after stop, got z={z}"
        return True, ""
    results.append(check(
        "scene.get_prim_info shows cube back at spawn Z after stop_simulation",
        resp, _back_at_spawn,
    ))

    # 5. URDF import round-trip is skipped because it requires a local URDF
    #    file in a known location — verified separately in the demo.

    # 6. Sensor smoke (camera only; lidar configs vary by Isaac Sim build)
    if engine in ("physx", "newton"):
        resp = send(args.host, args.port, "sensors.create_camera", {
            "prim_path": "/World/SmokeCamera",
            "position": [3.0, 0.0, 2.0],
            "resolution": [320, 240],
        })
        results.append(check("sensors.create_camera (experimental.rtx.RtxCamera)", resp))

    print()
    passed = sum(1 for r in results if r)
    print(f"{passed}/{len(results)} checks passed.")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
