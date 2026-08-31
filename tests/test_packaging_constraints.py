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

"""The published wheel must constrain its one runtime dependency.

`mcp[cli]` was unbounded, so `pip install isaacsim-mcp-server` resolved mcp 2.x,
where FastMCP was renamed to MCPServer. Every fresh install died on import:

    ModuleNotFoundError: No module named 'mcp.server.fastmcp'. This is mcp 2.x,
    where FastMCP was renamed to MCPServer ... or pin 'mcp<2'

Nothing in the repo could see it: uv.lock pins 1.28.1 so `uv run` works, and CI
installs only ruff and pytest by pip, never the package itself. It is only
visible by installing the built wheel into a clean environment.

The bounds are measured, not guessed: 1.9.0, 1.15.0 and 1.28.1 each serve all
42 tools through the installed console script; 2.1.1 cannot be imported.
"""

import os
import re

PYPROJECT = os.path.join(os.path.dirname(__file__), "..", "pyproject.toml")


def _dependency(name):
    """The dependency entry for `name`.

    Collect lines until the array's closing bracket on its own line -- splitting
    on the first "]" truncates inside "mcp[cli]" and silently returns a partial
    entry, which is a test that passes for the wrong reason.
    """
    with open(PYPROJECT) as f:
        lines = f.read().splitlines()
    inside = False
    for line in lines:
        if line.startswith("dependencies = ["):
            inside = True
            continue
        if inside:
            if line.strip() == "]":
                break
            entry = line.strip().rstrip(",").strip('"')
            if entry.startswith(name):
                return entry
    raise AssertionError(f"{name} is not declared in [project].dependencies")


def test_mcp_excludes_the_2x_line():
    """mcp 2.x renamed FastMCP; without an upper bound every fresh install breaks."""
    dep = _dependency("mcp")
    assert "<2" in dep, f"mcp must exclude the 2.x line, got {dep!r}"


def test_mcp_has_a_lower_bound():
    """An unbounded floor lets a resolver pick a version predating FastMCP."""
    dep = _dependency("mcp")
    assert re.search(r">=\s*\d+\.\d+", dep), f"mcp needs a lower bound, got {dep!r}"
