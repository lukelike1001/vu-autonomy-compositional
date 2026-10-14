#!/usr/bin/env python3
"""Which cells of a room the Neo 2 host would actually fly: the room's FLYABLE region, at the host's own margin.

The grid (``lab_grid.json``) calls a cell free when every 0.05 m cell inside it is free in the safenav footprint.
The flight host (``vu-autonomy-neo2/tools/neo2_closedloop.py``) is stricter: before it flies a pattern it measures
every straight leg against the survey cloud, in the band the props sweep at the flight altitude, and LANDS instead
of flying when any point is closer than ``--clearance-m`` (0.60 m: the aircraft plus its hover wander). A route the
grid allows can therefore be refused in the air (2026-10-14: 0 of 69 Room 1 routes cleared 0.60 m).

This applies the host's own test, with the host's own code (``vu_autonomy_core.freespace.ClearanceMap``, the
``core`` pinned by the neo2 checkout) and the host's own cloud:

* a cell is flyable when its centre is at least ``--margin`` from structure;
* a move between two 4-adjacent flyable cells is flyable when the whole straight leg between their centres is (sampled
  every 0.05 m, as the host's ``_path_clearance`` does);
* the region is the flyable cells reachable from ``--anchor`` (the take-off pad) by flyable moves, and its goal is
  the region's cell nearest the doorway.

It refuses to write a region in which two 4-adjacent cells are joined by a leg that is not flyable: then "always on a
region cell, moving one cell per tick" would not imply "every flown leg is clear", which is what the NSBT's safety
proof is meant to say.

Runs on the flight box (the cloud and the neo2 checkout are there), in the flight venv:

    PYTHONDONTWRITEBYTECODE=1 ~/venvs/neo2-flight/bin/python grid/flyable.py      # -> grid/room1_flyable.json
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True                     # never write __pycache__ into the neo2 checkout
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from lab_grid import load_lab_grid  # noqa: E402

NEO2 = Path(os.environ.get("NEO2_REPO", "~/ansr-final/vu-autonomy-neo2")).expanduser()
CLOUD = Path("~/tello-sites/verivital-lab-v6.2/cloud_5mm.npz").expanduser()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--room", default="room1")
    ap.add_argument("--anchor", default="8,15", help="the take-off cell x,y (default: the Neo 2 pad (E 1.47, N 0.64: 28 cm from floor tag 60 toward wall tag 15, nose at wall tag 36))")
    ap.add_argument("--alt", type=float, default=1.55, help="flight altitude, m (the band the props sweep)")
    ap.add_argument("--margin", type=float, default=0.60, help="the host's --clearance-m")
    ap.add_argument("--cloud", type=Path, default=CLOUD)
    ap.add_argument("--out", type=Path, default=None, help="default: grid/<room>_flyable.json")
    a = ap.parse_args()

    sys.path.insert(0, str(NEO2 / "core"))
    from vu_autonomy_core.freespace import ClearanceMap
    lab = load_lab_grid()
    E0, N0 = lab.raw["conventions"]["sw_corner_EN_m"]
    c = lab.raw["params"]["cell_m"]
    centre = lambda p: (E0 + (p[0] + 0.5) * c, N0 + (p[1] + 0.5) * c)
    cm = ClearanceMap.from_cloud_npz(str(a.cloud), alt_m=a.alt)
    room = lab.regions[a.room]
    clear = {p: round(cm.clearance_m(*centre(p)), 3) for p in room}
    ok = {p for p, w in clear.items() if w >= a.margin}

    def leg(p, q):
        return min(cm.clearance_m(e, n) for e, n in cm.densify([centre(p), centre(q)], 0.05))

    nbrs = lambda p: [(p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)]
    legs = {(p, q): round(leg(p, q), 3) for p in ok for q in nbrs(p) if q in ok and p < q}
    anchor = tuple(int(v) for v in a.anchor.split(","))
    if anchor not in ok:
        raise SystemExit(f"the anchor {anchor} is not flyable ({clear.get(anchor)} m from structure)")
    region, stack = {anchor}, [anchor]
    while stack:
        p = stack.pop()
        for q in nbrs(p):
            key = (min(p, q), max(p, q))
            if q in ok and q not in region and legs.get(key, 0) >= a.margin:
                region.add(q)
                stack.append(q)
    blocked = [[list(p), list(q), w] for (p, q), w in legs.items() if p in region and q in region and w < a.margin]
    if blocked:
        raise SystemExit(f"adjacent region cells joined by a leg under {a.margin} m: {blocked[:4]}: not written")
    door = centre(lab.door)
    goal = min(region, key=lambda p: (centre(p)[0] - door[0]) ** 2 + (centre(p)[1] - door[1]) ** 2)
    commit = subprocess.run(["git", "-C", str(NEO2), "rev-parse", "--short", "HEAD"], capture_output=True,
                            text=True).stdout.strip()
    core_commit = subprocess.run(["git", "-C", str(NEO2 / "core"), "rev-parse", "--short", "HEAD"],
                                 capture_output=True, text=True).stdout.strip()
    out = a.out or HERE / f"{a.room}_flyable.json"
    out.write_text(json.dumps({
        "schema": "lab-grid-flyable/1",
        "claim": "The cells of one room whose centres, and the legs between whose 4-adjacent centres, the Neo 2 host's "
                 "pre-arm clearance test accepts at this altitude and margin. A statement about the survey cloud, "
                 "not about the room as it is today.",
        "room": a.room, "alt_m": a.alt, "margin_m": a.margin, "anchor": list(anchor), "goal": list(goal),
        "goal_rule": "the region's cell nearest the doorway's centre",
        "cells": sorted(list(p) for p in region),
        "provenance": {"cloud": str(a.cloud), "cloud_sha256": hashlib.sha256(a.cloud.read_bytes()).hexdigest(),
                       "band_m": [round(cm.band[0], 3), round(cm.band[1], 3)],
                       "clearance_code": f"vu-autonomy-neo2 {commit}, core {core_commit}: "
                                         "vu_autonomy_core.freespace.ClearanceMap",
                       "lab_grid_sha256": hashlib.sha256((HERE / "lab_grid.json").read_bytes()).hexdigest(),
                       "made": dt.datetime.now().isoformat(timespec="seconds")},
        "centre_clearance_m": {f"{p[0]},{p[1]}": w for p, w in sorted(clear.items())},
        "leg_clearance_m": {f"{p[0]},{p[1]}-{q[0]},{q[1]}": w for (p, q), w in sorted(legs.items())},
    }, indent=1) + "\n")
    print(f"{out.name}: {len(region)} flyable cells reachable from {anchor} (of {len(ok)} clear centres, "
          f"{len(room)} {a.room} cells) at {a.margin} m, alt {a.alt} m; goal {goal} (nearest the doorway)")


if __name__ == "__main__":
    main()
