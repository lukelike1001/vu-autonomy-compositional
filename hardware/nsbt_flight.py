#!/usr/bin/env python3
"""Fly an NSBT example's route on the Neo 2: the BehaVerify tree decides, the lab's closed-loop host flies.

The tree reaches the aircraft as a ROUTE TABLE (``examples/<name>/routes.json``, written by
``examples/route_table.py`` with the BehaVerify-generated tree on the flight box): for every start cell, the cells
the tree drives the drone through, one per tick. The host is the lab's own ``tools/neo2_closedloop.py live`` in
``--mode square``, unchanged, with ONE substitution: its two pattern functions (``_build_pattern``, what it flies,
and ``_pattern_points``, what every pre-arm check scores) return the route the table holds for the cell of the
aircraft's first position fix. So the flight keeps everything the host does: warm and take-off, the tag localizer,
the fixed heading (``--yaw-mode fixed``: it never turns), the clearance gate at 0.60 m and the tag-visibility gate
(either refusal LANDS), the stale-pose hold and abort, stall handling, the recording and the landing.

A fix outside the table (not on a flyable cell) gives a one-point pattern at the fix: the host holds there and lands.

    PY=~/venvs/neo2-flight/bin/python
    $PY hardware/nsbt_flight.py sitl examples/room1_nn1             # fly the route in the lab's simulator
    $PY hardware/nsbt_flight.py host examples/room1_nn1             # DRY RUN on the aircraft: no arm, no take-off
    $PY hardware/nsbt_flight.py host examples/room1_nn1 --arm       # THE FLIGHT (use hardware/neo2_nsbt.py)

Runs on the flight box. Never writes into the neo2 checkout (no __pycache__ either).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import random
import sys
import threading
import time
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
NEO2 = Path(os.environ.get("NEO2_REPO", "~/ansr-final/vu-autonomy-neo2")).expanduser()
SIM = Path(os.environ.get("NEO2_SIM_REPO", "~/ansr-final/vu-autonomy-sim")).expanduser()
SITE = Path("~/tello-sites/verivital-lab-v6.2").expanduser()
CLOUD, TAGS = SITE / "cloud_5mm.npz", SITE / "tags" / "tag_scale_v5_ingrid_safe.json"
#: the vu-lab options of the lab's own S19 line (read out of fly_scenario.sh by its SITL driver, 2026-10-14), minus
#: what is the truck's (explore, follow, stare, online planning). The box is Friday's flight box.
LAB_ARGS = ["--gimbal-deg", "-32", "--heading-deg", "best", "--yaw-mode", "fixed", "--plan", "static",
            "--plan-profile", "neo2-lab", "--plan-box=-1.6,-3,5.2,1.2", "--site-id", "vu-lab",
            "--survey", "2026-09-02-v6.2", "--tag-map", str(TAGS), "--cloud", str(CLOUD), "--no-calib"]


def _paths(sim: bool = False):
    for p in ([str(SIM)] if sim else []) + [str(NEO2 / "core"), str(NEO2 / "tools"), str(NEO2)]:
        if p not in sys.path:
            sys.path.insert(0, p)


# ------------------------------------------------------------------ the route table

class Routes:
    def __init__(self, example: Path):
        self.example = Path(example).resolve()
        self.ex = json.loads((self.example / "example.json").read_text())
        self.d = json.loads((self.example / "routes.json").read_text())
        g = self.d["grid"]
        self.E0, self.N0 = g["sw_corner_EN_m"]
        self.c = g["cell_m"]

    def cell_of(self, e: float, n: float) -> tuple[int, int]:
        return int(math.floor((e - self.E0) / self.c)), int(math.floor((n - self.N0) / self.c))

    def centre(self, cell) -> tuple[float, float]:
        return self.E0 + (cell[0] + 0.5) * self.c, self.N0 + (cell[1] + 0.5) * self.c

    def lookup(self, e: float, n: float):
        """(cell, route) for the cell containing (e, n), or (cell, None) if that cell is not in the table."""
        cell = self.cell_of(e, n)
        return cell, self.d["routes"].get(f"{cell[0]},{cell[1]}")


# ------------------------------------------------------------------ the substitution into the lab's host

def patch_host(H, routes: Routes, log=print):
    """Make the host's square mode fly the tree's route. Both pattern functions are replaced, so the flown pattern and
    the pattern every pre-arm check scores are the same points, as the host itself requires."""
    chosen = {}

    def route_points(e0, n0):
        cell, r = routes.lookup(e0, n0)
        if r is None:
            if "cell" not in chosen:
                log(f"  NSBT: the fix ({e0:.2f}, {n0:.2f}) is in cell {cell}, NOT a flyable start of the table: "
                    f"holding at the fix, then landing")
            chosen.update(cell=cell, route=None)
            return None
        if chosen.get("cell") != cell:
            log(f"  NSBT: fix ({e0:.2f}, {n0:.2f}) -> cell {cell}; the tree's route: "
                f"{' '.join(r['acts'])} -> {r['cells'][-1]} ({len(r['cells']) - 1} moves)")
            chosen.update(cell=cell, route=r)
        return [tuple(p) for p in r["en"]]

    orig_build, orig_points = H._build_pattern, H._pattern_points

    def build(args, e0, n0, heading):
        if args.mode != "square":
            return orig_build(args, e0, n0, heading)
        pts = route_points(e0, n0)
        if pts is None:
            return [(e0, n0)], 1.0, f"NSBT: fix outside the table, hold at ({e0:.2f},{n0:.2f})"
        return pts, 1.0, f"NSBT {routes.example.name}: {len(pts) - 1} moves from cell {chosen['cell']}"

    def points(args, e0, n0, yaw_rad):
        if args.mode != "square":
            return orig_points(args, e0, n0, yaw_rad)
        pts = route_points(e0, n0)
        return [(e0, n0)] + (pts or [])

    H._build_pattern, H._pattern_points = build, points
    return chosen


def cmd_host(a):
    """The lab's host, live, with the route substituted. Without --arm it is the host's own dry run."""
    _paths()
    routes = Routes(a.example)
    fl = routes.ex.get("flight", {})
    alt = str(a.alt or fl.get("alt_m", 1.55))
    os.chdir(NEO2)
    import neo2_closedloop as H
    chosen = patch_host(H, routes)
    argv = ["live", "--mode", "square", "--alt", alt, "--max-alt", "2.2", "--max-s", str(a.max_s),
            "--clearance-m", str(fl.get("clearance_m", 0.60)), "--warm", "--run-dir", str(a.run_dir), *LAB_ARGS]
    if a.arm:
        argv.append("--arm")
    print(f"NSBT host ({'ARMED' if a.arm else 'DRY RUN, no arm'}): neo2_closedloop.py {' '.join(argv)}", flush=True)
    print(f"  routes: {routes.example / 'routes.json'} (tree {routes.d['provenance']['tree_sha256'][:12]}, "
          f"behaverify {routes.d['provenance']['behaverify']})", flush=True)
    sys.argv = ["neo2_closedloop.py", *argv]
    rc = H.main()
    print(f"NSBT host: exit {rc}; route from cell {chosen.get('cell')}: "
          f"{(chosen.get('route') or {}).get('acts')}", flush=True)
    return rc or 0


# ------------------------------------------------------------------ the lab's simulator

def cmd_sitl(a):
    """Fly the route from the example's start cell through the host's own run_closed_loop against the
    vu-autonomy-sim plant, as analysis/neo2-lab-search-20260919/sitl_search.py does. The pose is truth + declared
    noise, so this tests the ROUTE and the CONTROL, not the tag localizer."""
    _paths(sim=True)
    import neo2_closedloop as H
    import vu_autonomy_core as core
    from vu_autonomy_core.freespace import ClearanceMap
    from vu_autonomy_sim.sitl import serve
    from neo2.comms.link import open_link
    from neo2.guidance import ClosedLoopGuide
    from neo2.perception.localize import office_tag_map
    from types import SimpleNamespace

    routes = Routes(a.example)
    fl = routes.ex.get("flight", {})
    alt = a.alt or fl.get("alt_m", 1.55)
    start_cell = tuple(int(v) for v in (a.start or ",".join(map(str, fl["start_cell"]))).split(","))
    r = routes.d["routes"][f"{start_cell[0]},{start_cell[1]}"]
    pts = [tuple(p) for p in r["en"]]
    start = routes.centre(start_cell)
    cm = ClearanceMap.from_cloud_npz(str(CLOUD), alt_m=alt)
    tmap = office_tag_map(str(TAGS))
    cam = H._planner_camera(SimpleNamespace(gimbal_deg=-32.0, plan_hfov=85.6))
    yaw, _, _ = core.best_heading(tmap, pts, alt, cam["pitch_rad"], cam["hfov_rad"], cam["vfov_rad"],
                                  fx_px=cam["fx_px"])
    rng = random.Random(a.seed)
    sock = f"/tmp/nsbt-sitl-{os.getpid()}.sock"
    srv, plant = serve(sock, "neo2", log=lambda *x: None)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    track, t0 = [], time.monotonic()

    class TruthPose:
        def pose(self):
            t = link.truth
            track.append((time.monotonic() - t0, t["e"], t["n"], t["u"], t["yaw_deg"]))
            return (t["e"] + rng.gauss(0, 0.03), t["n"] + rng.gauss(0, 0.03), t["u"],
                    math.radians(t["yaw_deg"] + rng.gauss(0, 1.0)), 0.02)

        def done(self):
            return False

    try:
        link = open_link("sitl", socket_path=sock)
        link.connect()
        plant.e, plant.n = start
        plant.yaw = math.degrees(yaw)
        link.takeoff()
        for _ in range(int(alt / 0.012 / 30) + 60):
            link.rc(0, 0, 40, 0)
            time.sleep(0.01)
            if link.latest.height_m and link.latest.height_m >= alt:
                break
        link.hover()
        res = H.run_closed_loop(TruthPose(), pts, ClosedLoopGuide(alt, yaw), link=link, recorder=None,
                                ctrl_hz=10.0, dwell_s=1.0, dry_run=False, max_s=a.max_s, log=lambda *x: None)
        link.land()
        link.close()
    finally:
        srv.shutdown()
        srv.server_close()
        if os.path.exists(sock):
            os.unlink(sock)

    safe = {tuple(c) for c in routes.d["safe_cells"]}
    goal = routes.centre(tuple(routes.d["goal"]))
    clear = [cm.clearance_m(e, n) for _, e, n, _, _ in track]
    on_safe = sum(1 for _, e, n, _, _ in track if routes.cell_of(e, n) in safe) / max(1, len(track))
    yaws = [y for *_, y in track]
    end = track[-1]
    out = {"example": routes.example.name, "start_cell": list(start_cell), "acts": r["acts"],
           "heading_deg": round(math.degrees(yaw), 1), "alt_m": alt, "ended": res.get("ended"),
           "waypoints_reached": res.get("waypoints_reached"), "n_waypoints": len(pts),
           "elapsed_s": res.get("elapsed_s"), "final_to_goal_m": round(math.hypot(end[1] - goal[0], end[2] - goal[1]), 3),
           "min_track_clearance_m": round(min(clear), 3), "track_on_safe_cells": round(on_safe, 3),
           "yaw_spread_deg": round(max(yaws) - min(yaws), 2), "samples": len(track),
           "tier": "SITL: vu-autonomy-sim surrogate plant, pose = truth + noise; gains declared, not measured"}
    ok = (out["ended"] == "completed" and out["final_to_goal_m"] <= 0.25 and out["min_track_clearance_m"] >= 0.50)
    out["passed"] = ok
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps({**out, "track": [[round(v, 3) for v in s] for s in track]}, indent=1) + "\n")
    print(json.dumps(out, indent=1))
    print("SITL PASS" if ok else "SITL FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("host", help="the lab's host, live, flying the route (dry run unless --arm)")
    p.add_argument("example", type=Path)
    p.add_argument("--arm", action="store_true")
    p.add_argument("--alt", type=float, default=None)
    p.add_argument("--max-s", type=float, default=90.0)
    p.add_argument("--run-dir", type=Path, default=Path(f"~/tello-runs/nsbt-{dt.date.today()}").expanduser())
    p = sub.add_parser("sitl", help="fly the route in the lab's simulator")
    p.add_argument("example", type=Path)
    p.add_argument("--start", default=None, help="start cell x,y (default: the example's flight start)")
    p.add_argument("--alt", type=float, default=None)
    p.add_argument("--max-s", type=float, default=90.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None, help="write the result and the truth track here (JSON)")
    a = ap.parse_args()
    return {"host": cmd_host, "sitl": cmd_sitl}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
