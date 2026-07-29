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

"""discover_robots walks the asset server concurrently."""

import ast
import os

V5 = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
    "v5.py",
)


def _discover_src():
    with open(V5) as f:
        text = f.read()
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "discover_robots":
            return ast.get_source_segment(text, node)
    raise AssertionError("discover_robots not found")


def test_walk_is_concurrent():
    """~150 sequential listings cost ~28 s on a cold cache and block kit's main
    loop for the whole time. The calls are latency bound, so they must overlap."""
    src = _discover_src()
    assert "ThreadPoolExecutor" in src


def test_walk_falls_back_to_sequential():
    """If threads are unavailable the walk must still work, just slower."""
    src = _discover_src()
    code = "\n".join(line.split("#", 1)[0] for line in src.splitlines())
    assert code.count("_list_dir(p) for p in paths") >= 2, "needs a sequential fallback path"


def test_ordering_preserved_for_key_preference():
    """Results are zipped back against the input order, because the 'shorter
    filename wins' rule depends on deterministic iteration order."""
    src = _discover_src()
    assert "zip(pairs, model_files)" in src
    assert "zip(mfr_names, mfr_models)" in src
