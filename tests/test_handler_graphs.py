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

"""Structural tests for the action-graph handler inline_script path."""

import ast
import os

HANDLERS = os.path.join(
    os.path.dirname(__file__), "..", "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension", "handlers",
)


def _handler_src():
    with open(os.path.join(HANDLERS, "graphs.py")) as f:
        return f.read()


def test_create_action_graph_accepts_inline_script_param():
    tree = ast.parse(_handler_src())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "create_action_graph":
            arg_names = {a.arg for a in node.args.args} | {a.arg for a in node.args.kwonlyargs}
            assert "inline_script" in arg_names
            return
    raise AssertionError("create_action_graph not found")


def test_inline_script_sets_script_and_disables_usepath():
    src = _handler_src()
    # inline path builds the same OnPlaybackTick -> ScriptNode pair and sets script inline
    assert "inline_script" in src
    assert "inputs:script" in src
    assert "inputs:usePath" in src
