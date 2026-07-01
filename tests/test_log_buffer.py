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

"""Unit tests for the get_isaac_logs buffer selection logic."""

from isaac_sim_mcp_extension.handlers import simulation as sim


def test_select_logs_since_last_play_filters_to_current_run():
    buf = ["a", "b", "c", "d"]
    # Play happened after 'b' -> boundary index 2
    assert sim._select_logs(buf, boundary=2, since_last_play=True, count=100) == ["c", "d"]


def test_select_logs_all_when_not_scoped():
    buf = ["a", "b", "c", "d"]
    assert sim._select_logs(buf, boundary=2, since_last_play=False, count=100) == ["a", "b", "c", "d"]


def test_select_logs_respects_count():
    buf = ["a", "b", "c", "d"]
    assert sim._select_logs(buf, boundary=0, since_last_play=True, count=2) == ["c", "d"]


def test_get_logs_default_is_non_destructive(monkeypatch):
    monkeypatch.setattr(sim, "_log_buffer", ["x", "y"], raising=False)
    monkeypatch.setattr(sim, "_play_boundary", 0, raising=False)
    monkeypatch.setattr(sim, "_ensure_log_listener", lambda: None)
    result = sim.get_logs(adapter=None)          # defaults: clear=False
    assert result["logs"] == ["x", "y"]
    assert sim._log_buffer == ["x", "y"]          # buffer intact


def test_append_and_mark_boundary_scopes_new_run(monkeypatch):
    monkeypatch.setattr(sim, "_log_buffer", [], raising=False)
    monkeypatch.setattr(sim, "_play_boundary", 0, raising=False)
    monkeypatch.setattr(sim, "_ensure_log_listener", lambda: None)
    sim.append_log("[PRINT] old")
    sim.mark_play_boundary()
    sim.append_log("[PRINT] new")
    result = sim.get_logs(adapter=None, since_last_play=True)
    assert result["logs"] == ["[PRINT] new"]
