# MIT License
#
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

"""Sensor handler behaviour that the integration tests cannot pin down.

The live capture test accepts either outcome (`assert resp["status"] in
("success", "error")`), so a capture that never produces a frame passes it. These
tests hold the contract instead.
"""

from __future__ import annotations

from isaac_sim_mcp_extension.handlers.sensors import capture_image


class _Frame:
    """Minimal stand-in for the ndarray the adapter returns (numpy is stubbed offline)."""

    def __init__(self, shape):
        self.shape = shape
        size = 1
        for dim in shape:
            size *= dim
        self.size = size


class _Adapter:
    def __init__(self, image):
        self._image = image
        self.calls = []

    def capture_camera_image(self, prim_path):
        self.calls.append(prim_path)
        return self._image


def test_capture_reports_an_error_when_no_frame_is_available():
    """An empty array means "no frame", and must not be reported as success.

    On 6.0.1 the step-only debug loop never plays the timeline, so Replicator's
    orchestrator stays STOPPED and every capture came back empty — while the
    tool answered {"status": "success", "shape": [0]}.
    """
    result = capture_image(_Adapter(_Frame((0,))), prim_path="/World/Cam")

    assert result["status"] == "error"
    assert "/World/Cam" in result["message"]
    # The message has to say what to do about it, not just that it failed.
    assert "playing" in result["message"]


def test_capture_reports_an_error_when_the_adapter_returns_none():
    result = capture_image(_Adapter(None), prim_path="/World/Cam")

    assert result["status"] == "error"


def test_capture_succeeds_with_a_real_frame():
    frame = _Frame((480, 640, 3))

    result = capture_image(_Adapter(frame), prim_path="/World/Cam")

    assert result["status"] == "success"
    assert result["shape"] == [480, 640, 3]


def test_empty_frame_is_never_written_to_disk(tmp_path):
    """With output_path set, an empty array used to reach Image.fromarray."""
    out = tmp_path / "shot.png"

    result = capture_image(_Adapter(_Frame((0,))), prim_path="/World/Cam", output_path=str(out))

    assert result["status"] == "error"
    assert not out.exists()
