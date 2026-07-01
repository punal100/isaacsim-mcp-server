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

"""Substring checks on MCP tool docstrings and the server instruction block."""

import os

TOOLS_DIR = os.path.join(os.path.dirname(__file__), "..", "isaac_mcp", "tools")
SERVER_PY = os.path.join(os.path.dirname(__file__), "..", "isaac_mcp", "server.py")


def _read_tool_source(filename):
    with open(os.path.join(TOOLS_DIR, filename)) as f:
        return f.read()


def _read_server_source():
    with open(SERVER_PY) as f:
        return f.read()


def test_create_object_documents_scale_multiplier():
    src = _read_tool_source("objects.py")
    # scale= is a raw multiplier of the primitive's native size
    assert "native size" in src
    assert "2 m" in src or "2m" in src           # native size of Cube/Sphere/etc
    assert "scale=0.5" in src                     # worked example -> 1 m
    assert "size=" in src                         # steer to size= for absolute meters
