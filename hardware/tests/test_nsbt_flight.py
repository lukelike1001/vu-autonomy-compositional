"""The NSBT flight pieces that decide what the aircraft flies, tested without the host or an aircraft:
the route table lookup, the substitution into the host's two pattern functions, and the flown-cells summary."""
from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))
import nsbt_flight as F  # noqa: E402
import neo2_nsbt as N  # noqa: E402

EXAMPLE = ROOT / "examples" / "room1_nn1"


@pytest.fixture
def routes():
    return F.Routes(EXAMPLE)


def test_the_table_is_the_trees_and_every_route_stays_flyable(routes):
    safe = {tuple(c) for c in routes.d["safe_cells"]}
    goal = tuple(routes.d["goal"])
    assert routes.d["routes"], "empty table"
    for key, r in routes.d["routes"].items():
        assert tuple(r["cells"][0]) == tuple(int(v) for v in key.split(","))
        assert tuple(r["cells"][-1]) == goal and r["acts"][-1] == "XX"
        assert all(tuple(c) in safe for c in r["cells"])
        for a, b, act in zip(r["cells"], r["cells"][1:], r["acts"]):          # one cell per tick, as the act says
            assert (b[0] - a[0], b[1] - a[1]) == {"We": (-1, 0), "Ea": (1, 0), "No": (0, 1), "So": (0, -1)}[act]
        assert all(abs(e - routes.centre(c)[0]) < 1e-3 and abs(n - routes.centre(c)[1]) < 1e-3
                   for (e, n), c in zip(r["en"], r["cells"]))


def test_a_fix_on_the_neo2_pad_flies_its_route(routes):
    cell, r = routes.lookup(1.47, 0.64)                            # the Neo 2 pad, LAB_HOWTO.md section 5
    assert cell == (8, 15) and r["acts"] == ["We", "So", "So", "So", "XX"]
    assert tuple(routes.ex["flight"]["start_cell"]) == (8, 15)


def fake_host():
    calls = []
    H = SimpleNamespace(_build_pattern=lambda *a: calls.append("orig_build") or ([(9, 9)], 1.0, "orig"),
                        _pattern_points=lambda *a: calls.append("orig_points") or [(9, 9)])
    return H, calls


def test_square_mode_flies_the_route_and_every_check_scores_the_same_points(routes):
    H, calls = fake_host()
    chosen = F.patch_host(H, routes, log=lambda *a: None)
    e0, n0 = routes.centre((5, 14))
    pts, dwell, label = H._build_pattern(SimpleNamespace(mode="square"), e0, n0, 0.3)
    r = routes.d["routes"]["5,14"]
    assert pts == [tuple(p) for p in r["en"]] and dwell == 1.0 and "NSBT" in label
    assert H._pattern_points(SimpleNamespace(mode="square"), e0, n0, 0.3) == [(e0, n0)] + pts
    assert chosen["cell"] == (5, 14) and not calls


def test_a_fix_off_the_table_holds_and_lands(routes):
    H, _ = fake_host()
    F.patch_host(H, routes, log=lambda *a: None)
    e0, n0 = routes.centre((9, 11))                                # Room 1, but not flyable
    pts, _, label = H._build_pattern(SimpleNamespace(mode="square"), e0, n0, 0.0)
    assert pts == [(e0, n0)] and "outside the table" in label
    assert H._pattern_points(SimpleNamespace(mode="square"), e0, n0, 0.0) == [(e0, n0)]


def test_other_modes_are_the_hosts_own(routes):
    H, calls = fake_host()
    F.patch_host(H, routes, log=lambda *a: None)
    H._build_pattern(SimpleNamespace(mode="hold"), 0, 0, 0)
    H._pattern_points(SimpleNamespace(mode="lawnmower"), 0, 0, 0)
    assert calls == ["orig_build", "orig_points"]


def test_the_flown_cells_summary(tmp_path, routes):
    rows = [{"kind": "state", "h": 0}, {"kind": "state", "h": 156}]
    for c in [(5, 14), (5, 14), (6, 14), (7, 14), (7, 13), (7, 12)]:
        e, n = routes.centre(c)
        rows.append({"kind": "ekf", "x": e, "y": n, "z": 1.55, "sec_since_fix": 0.1})
    rows.append({"kind": "ekf", "x": 0.0, "y": 0.0, "z": 0.2, "sec_since_fix": 0.1})          # on the ground: ignored
    rows.append({"kind": "ekf", "x": 9.0, "y": 9.0, "z": 1.5, "sec_since_fix": 5.0})          # stale fix: ignored
    (tmp_path / "flight.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    v = N.visited_cells(tmp_path, routes)
    assert v["cells_visited"] == [[5, 14], [6, 14], [7, 14], [7, 13], [7, 12]]
    assert v["max_height_m"] == 1.56 and v["ekf_rows_used"] == 6
