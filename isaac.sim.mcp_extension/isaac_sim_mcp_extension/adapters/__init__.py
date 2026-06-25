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

"""Isaac Sim version adapters."""

from __future__ import annotations

import os
import re


def _detect_isaacsim_major_version() -> int:
    """Return the major version of the running Isaac Sim runtime, or 0 on failure."""
    try:
        from isaacsim.core.version import get_version  # type: ignore

        # get_version() returns a string in 5.x ("5.1.0") and a tuple in 6.0
        # (("6.0.0", "rc.59", "6", "0", "0", ...)). Pick the first element when
        # given a sequence, otherwise treat as a string.
        version_value = get_version()
        if isinstance(version_value, (list, tuple)) and version_value:
            version_str = str(version_value[0])
        else:
            version_str = str(version_value)
        match = re.match(r"^(\d+)\.", version_str)
        if match:
            return int(match.group(1))
    except Exception:
        pass

    # Fallback: read $ISAAC_PATH/VERSION (set by Isaac Sim launcher) or sibling VERSION file
    for env_var in ("ISAAC_PATH", "ISAACSIM_PATH"):
        root = os.environ.get(env_var)
        if not root:
            continue
        version_file = os.path.join(root, "VERSION")
        if os.path.isfile(version_file):
            try:
                with open(version_file) as f:
                    text = f.read().strip()
                match = re.match(r"^(\d+)\.", text)
                if match:
                    return int(match.group(1))
            except OSError:
                continue
    return 0


def get_adapter():
    """Return the appropriate adapter for the current Isaac Sim version.

    Selects ``IsaacAdapterV6`` when the runtime reports major version >= 6,
    ``IsaacAdapterV5`` otherwise (including detection failure, which is safe
    under 5.1 and produces a clear ImportError under 6.0 Newton — directing
    the user to upgrade).
    """
    if _detect_isaacsim_major_version() >= 6:
        from .v6 import IsaacAdapterV6

        return IsaacAdapterV6()
    from .v5 import IsaacAdapterV5

    return IsaacAdapterV5()
