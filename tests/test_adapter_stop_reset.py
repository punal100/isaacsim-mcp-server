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

"""Verify stop() performs a physics reset (AST) — live behaviour is in smoke_test_v6."""

import ast
import os

ADAPTERS = os.path.join(
    os.path.dirname(__file__), "..", "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension", "adapters",
)


def _stop_body_src(filename):
    with open(os.path.join(ADAPTERS, filename)) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "stop":
            return ast.get_source_segment(open(os.path.join(ADAPTERS, filename)).read(), node)
    return ""


def test_v6_stop_resets_physics():
    src = _stop_body_src("v6.py")
    assert "reset" in src.lower()
    assert "stop()" in src  # still stops the timeline first


def test_v5_stop_resets_physics():
    src = _stop_body_src("v5.py")
    assert "reset" in src.lower()
    assert "stop()" in src
