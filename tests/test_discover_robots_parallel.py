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


# ── Functional tests against a fake asset server ────────────────────────────


class _Entry:
    def __init__(self, name):
        self.relative_path = name


def _fake_client(tree):
    """Minimal omni.client stand-in over a {path: [names]} dict."""
    import types

    class _Result:
        OK = "OK"

    mod = types.SimpleNamespace(Result=_Result)

    def _list(path):
        key = path.rstrip("/") + "/"
        if key not in tree:
            return ("MISSING", [])
        return (_Result.OK, [_Entry(n) for n in tree[key]])

    mod.list = _list
    return mod


def _run_discovery(monkeypatch, tree):
    import sys
    import types

    fake = _fake_client(tree)
    monkeypatch.setitem(sys.modules, "omni.client", fake)
    # `import omni.client` reads the attribute off the parent package, so the
    # stub has to be attached there as well as in sys.modules.
    monkeypatch.setattr(sys.modules["omni"], "client", fake, raising=False)

    native = types.SimpleNamespace(get_assets_root_path=lambda: "ROOT")
    monkeypatch.setitem(sys.modules, "isaacsim.storage.native", native)
    if "isaacsim" in sys.modules:
        storage = getattr(sys.modules["isaacsim"], "storage", None)
        if storage is not None:
            monkeypatch.setattr(storage, "native", native, raising=False)

    from isaac_sim_mcp_extension.adapters.v5 import IsaacAdapterV5

    return IsaacAdapterV5().discover_robots()


def test_hidden_thumbnail_directories_are_not_robots():
    """Every manufacturer ships a .thumbs folder of <model>.thumb.usd previews.

    Unfiltered they registered as a robot literally named ".thumbs" whose
    asset_path pointed at a thumbnail — observed on Isaac Sim 5.1.
    """
    import pytest

    tree = {
        "ROOT/Isaac/Robots/": ["ANYbotics/", "Idealworks/"],
        "ROOT/Isaac/Robots/ANYbotics/": [".thumbs/", ".cache/", "anymal_c/"],
        "ROOT/Isaac/Robots/ANYbotics/.thumbs/": ["anymal_c.thumb.usd"],
        # A hidden directory holding a plain .usd — covered by the hidden-dir
        # rule rather than the .thumb.usd rule, so both guards are exercised.
        "ROOT/Isaac/Robots/ANYbotics/.cache/": ["scratch.usd"],
        "ROOT/Isaac/Robots/ANYbotics/anymal_c/": ["anymal_c.usd"],
        "ROOT/Isaac/Robots/Idealworks/": [".thumbs/", "iwhub/"],
        "ROOT/Isaac/Robots/Idealworks/.thumbs/": ["iw_hub.thumb.usd"],
        "ROOT/Isaac/Robots/Idealworks/iwhub/": ["iw_hub.usd"],
    }
    mp = pytest.MonkeyPatch()
    try:
        robots = _run_discovery(mp, tree)
    finally:
        mp.undo()

    assert ".thumbs" not in robots
    assert not [k for k in robots if k.startswith(".")]
    assert not [r for r in robots.values() if r["asset_path"].endswith(".thumb.usd")]
    assert set(robots) == {"anymal_c", "iwhub"}


def test_colliding_model_names_keep_a_consistent_record():
    """Two vendors can ship the same directory name.

    The "shorter filename wins" rule used to overwrite asset_path only, leaving
    a record that described one manufacturer while pointing at another's asset.
    """
    import pytest

    tree = {
        "ROOT/Isaac/Robots/": ["AlphaCorp/", "BetaCorp/"],
        "ROOT/Isaac/Robots/AlphaCorp/": ["shared/"],
        "ROOT/Isaac/Robots/AlphaCorp/shared/": ["a_very_long_name.usd"],
        "ROOT/Isaac/Robots/BetaCorp/": ["shared/"],
        "ROOT/Isaac/Robots/BetaCorp/shared/": ["b.usd"],
    }
    mp = pytest.MonkeyPatch()
    try:
        robots = _run_discovery(mp, tree)
    finally:
        mp.undo()

    rec = robots["shared"]
    assert f"/Isaac/Robots/{rec['manufacturer']}/" in rec["asset_path"], rec
    assert rec["manufacturer"] == "BetaCorp"  # shorter filename won
    assert rec["description"] == "BetaCorp shared"
