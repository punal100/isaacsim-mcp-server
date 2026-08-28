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

"""reload_script(module_name=...) must not re-run stale bytecode.

Measured live on 5.1 and 6.0: editing a module and reloading it re-executed the
previous contents while reporting "reloaded successfully", so the caller
believed an edit was live when the old code was still running. Deleting the
.pyc fixed it. This is issue #3 item 4 / #4 item 1, reopened as #27.
"""

import ast
import importlib
import os
import sys

from isaac_sim_mcp_extension.adapters.base import drop_stale_bytecode

ADAPTERS = os.path.join(
    os.path.dirname(__file__),
    "..",
    "isaac.sim.mcp_extension",
    "isaac_sim_mcp_extension",
    "adapters",
)


def test_drop_stale_bytecode_removes_the_cached_pyc(tmp_path):
    src = tmp_path / "reload_probe_mod.py"
    src.write_text("VERSION = 1\n")
    sys.path.insert(0, str(tmp_path))
    try:
        mod = importlib.import_module("reload_probe_mod")
        assert mod.VERSION == 1
        cache = importlib.util.cache_from_source(str(src))
        assert os.path.exists(cache), "import should have written bytecode to prime the test"

        drop_stale_bytecode(str(src))

        assert not os.path.exists(cache), "stale bytecode survived"
    finally:
        sys.path.remove(str(tmp_path))
        sys.modules.pop("reload_probe_mod", None)


def test_drop_stale_bytecode_tolerates_a_missing_cache(tmp_path):
    """A module that was never imported has no .pyc — that is not an error."""
    src = tmp_path / "never_imported.py"
    src.write_text("VERSION = 1\n")
    drop_stale_bytecode(str(src))  # must not raise


def test_reload_after_dropping_bytecode_sees_the_edit(tmp_path):
    """The end-to-end behaviour the tool promises: an on-disk edit takes effect."""
    src = tmp_path / "reload_edit_mod.py"
    src.write_text("VERSION = 1\n")
    sys.path.insert(0, str(tmp_path))
    try:
        mod = importlib.import_module("reload_edit_mod")
        assert mod.VERSION == 1

        # Same byte length as the original, which is what defeated the
        # mtime+size staleness check in the live reproduction.
        src.write_text("VERSION = 2\n")
        os.utime(src, (0, 0))  # force an mtime the cache would consider current

        drop_stale_bytecode(str(src))
        importlib.invalidate_caches()
        mod = importlib.reload(sys.modules["reload_edit_mod"])

        assert mod.VERSION == 2, "reload re-ran stale bytecode"
    finally:
        sys.path.remove(str(tmp_path))
        sys.modules.pop("reload_edit_mod", None)


def _adapter_src(name):
    with open(os.path.join(ADAPTERS, name)) as f:
        return f.read()


def test_both_adapters_drop_bytecode_before_reloading():
    """v5 and v6 carry the same reload path; a fix in one only is half a fix."""
    for name in ("v5.py", "v6.py"):
        tree = ast.parse(_adapter_src(name))
        called = {n.func.id for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
        assert "drop_stale_bytecode" in called, f"{name} reloads without dropping bytecode"
        assert "invalidate_caches" in _adapter_src(name), f"{name} never invalidates import caches"
