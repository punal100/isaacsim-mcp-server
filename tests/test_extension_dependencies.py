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

# SPDX-License-Identifier: MIT
"""The extensions each adapter imports must be declared, and declared optional.

`extension.toml` gates what the extension can import on each runtime. The V5
adapter's dependencies were dropped when the V6 experimental ones were added:
it works on a stock 5.1 app only because that app happens to enable them, and
on a leaner one v5.py raises ModuleNotFoundError at call time, which reaches the
caller as a bare handler error rather than a missing dependency.

They must be `optional`. Verified against both installs on this machine:

    ~/isaacsim-5.1.0/exts   has isaacsim.core.api / .prims / .utils,
                            isaacsim.sensors.camera / .rtx
    ~/isaacsim/exts (6.0.1) has none of them

so a hard dependency on any of them stops the extension loading on 6.0 —
exactly the failure isaacsim.sensors.experimental.rtx documents in reverse.
"""

import os
import re

import pytest

EXT = os.path.join(os.path.dirname(__file__), "..", "isaac.sim.mcp_extension")
TOML = os.path.join(EXT, "config", "extension.toml")

# Imported by v5.py, present only on 5.1.
V5_ONLY = [
    "isaacsim.core.api",
    "isaacsim.core.prims",
    "isaacsim.core.utils",
    "isaacsim.sensors.camera",
    "isaacsim.sensors.rtx",
]

# Imported by v6.py, present only on 6.0.
V6_ONLY = ["isaacsim.sensors.experimental.rtx"]


def _declared():
    """Map of declared dependency name -> True when marked optional."""
    with open(TOML) as f:
        block = f.read().split("[dependencies]", 1)[1].split("\n[", 1)[0]
    out = {}
    for line in block.splitlines():
        line = line.split("#", 1)[0].strip()
        m = re.match(r'"([^"]+)"\s*=\s*(\{.*\})', line)
        if m:
            out[m.group(1)] = bool(re.search(r"optional\s*=\s*true", m.group(2)))
    return out


@pytest.mark.parametrize("name", V5_ONLY + V6_ONLY)
def test_version_specific_extension_is_declared(name):
    assert name in _declared(), f"{name} is imported by an adapter but not declared in extension.toml"


@pytest.mark.parametrize("name", V5_ONLY + V6_ONLY)
def test_version_specific_extension_is_optional(name):
    assert _declared().get(name) is True, (
        f"{name} exists on only one runtime; a hard dependency stops the extension loading on the other"
    )
