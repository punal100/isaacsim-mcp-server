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

# The floor is where `instructions=` starts being honoured, measured against the
# built wheel with --no-deps so the version under test is the one installed:
#
#   mcp 1.2.0   imports, serves 42 tools, instructions DROPPED SILENTLY
#   mcp 1.3.0   imports, serves 42 tools, instructions present (3546 chars)
#
# Below 1.3.0 nothing raises. FastMCP.__init__(self, name=None, **settings)
# swallows the keyword, so the server starts and quietly loses the whole agent
# contract -- which is worse than the 2.x break, because that one at least fails
# loudly at import.
MIN_SUPPORTED = (1, 3, 0)


def _dependency(name, text=None):
    """The dependency entry for `name` from [project].dependencies.

    Two shapes defeated earlier versions of this helper, both found by review:

      * splitting on the first "]" truncates inside "mcp[cli]";
      * `startswith(name)` matches "mcp-extras", and an inline comment
        mentioning "<2" satisfied the bound check while the real entry was
        unbounded -- and this repo already writes trailing comments on
        dependency lines (ruff==0.16.1 carries one).

    So: strip comments, and match the distribution name on a boundary.
    tomllib is not an option -- CI runs Python 3.10.
    """
    if text is None:
        with open(PYPROJECT) as f:
            text = f.read()
    inside = False
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if line.startswith("dependencies = ["):
            inside = True
            continue
        if inside:
            if line.strip() == "]":
                break
            entry = line.strip().rstrip(",").strip('"').strip("'")
            if re.match(rf"{re.escape(name)}(\[|[<>=!~;\s]|$)", entry):
                return entry
    raise AssertionError(f"{name} is not declared in [project].dependencies")


def _floor(dep):
    """The declared lower bound as a tuple, or None when there is not one."""
    m = re.search(r">=\s*(\d+(?:\.\d+)*)", dep)
    return tuple(int(part) for part in m.group(1).split(".")) if m else None


def test_mcp_excludes_the_2x_line():
    """mcp 2.x renamed FastMCP; without an upper bound every fresh install breaks."""
    dep = _dependency("mcp")
    assert "<2" in dep, f"mcp must exclude the 2.x line, got {dep!r}"


def test_mcp_floor_is_at_least_the_version_that_honours_instructions():
    """A floor must exist AND be high enough to matter.

    Asserting only that some ">=" is present accepts ">=0.1", which reintroduces
    the silent instruction loss above. Compare numerically instead, so raising
    the floor stays fine and lowering it past the measured boundary fails.
    """
    dep = _dependency("mcp")
    floor = _floor(dep)
    assert floor is not None, f"mcp needs a lower bound, got {dep!r}"
    assert floor >= MIN_SUPPORTED, (
        f"mcp floor {floor} is below {MIN_SUPPORTED}, where `instructions=` is "
        f"silently dropped and the server loses its agent contract; got {dep!r}"
    )


def test_parser_is_not_fooled_by_a_comment_that_mentions_the_bound():
    """A trailing comment must not satisfy the bound check."""
    text = 'dependencies = [\n    "mcp[cli]>=1.9.0",  # never let this go <2\n]\n'
    assert _dependency("mcp", text) == "mcp[cli]>=1.9.0"


def test_parser_does_not_match_a_different_distribution_by_prefix():
    """ "mcp-extras" is not "mcp"."""
    text = 'dependencies = [\n    "mcp-extras>=1.0,<2",\n    "mcp[cli]>=1.9.0,<2",\n]\n'
    assert _dependency("mcp", text) == "mcp[cli]>=1.9.0,<2"


def test_parser_finds_a_bare_name_and_an_extras_form():
    for text, expected in (
        ('dependencies = [\n    "mcp",\n]\n', "mcp"),
        ('dependencies = [\n    "mcp[cli]>=1.3.0,<2",\n]\n', "mcp[cli]>=1.3.0,<2"),
    ):
        assert _dependency("mcp", text) == expected
