#!/usr/bin/env python3
"""Fly an NSBT example on the Neo 2: the operator's runner. The first flight of a BehaVerify tree on an aircraft.

    python3 hardware/neo2_nsbt.py --dry       # the host's dry run: aircraft on the pad, motors never start
    python3 hardware/neo2_nsbt.py             # THE FLIGHT: type "nsbt" when ready

It runs on the flight box (Taylor's Alienware), started by the operator, who stands where the aircraft can be caught.
In order, stopping at the first problem:

1. nobody else is flying from this box;
2. THE TREE, ON THE FLIGHT BOX: BehaVerify (``~/venvs/nsbt-bv``) generates the example's tree and runs it from every
   start cell (``examples/route_table.py``). The table must equal the committed ``routes.json``, the one the nuXmv
   proofs and the simulator runs were made against; otherwise it stops (regenerate, re-prove, re-simulate first);
3. the box joins the aircraft's Wi-Fi and 192.168.2.1 routes through it (``ROUTE-OK``);
4. ``--wait`` seconds after power-on (20 by default);
5. the lab's closed-loop host, ``tools/neo2_closedloop.py live --mode square``, flying the tree's route from the cell
   of its first position fix (``hardware/nsbt_flight.py host``). With ``--dry`` it is the host's own dry run (no arm,
   no take-off); otherwise it is launched only after the operator types ``nsbt``;
6. the box leaves the aircraft's Wi-Fi (always), and the run is summarised: the cells the aircraft's position passed
   through, against the tree's route.

LANDING. The host lands by itself at the end of the route, on a refusal (clearance, tag visibility), on a lost pose,
at --max-s and at 20 % battery. To land sooner: ONE Ctrl-C in this terminal (the host lands on Ctrl-C), or a hand
underneath it.

Records: the aircraft's run in ``~/tello-runs/nsbt-<date>/<run_id>/`` (a rundb root); this session's notes, the route
table made on the flight box and every output in ``~/neo2-sessions/<date>/``.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import time
from pathlib import Path

os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import neo2_hover as HV  # noqa: E402      the hover runner's tested pieces: Wi-Fi, streaming, the busy check
from nsbt_flight import Routes  # noqa: E402

FLIGHT_PY = str(Path("~/venvs/neo2-flight/bin/python").expanduser())
BV_PY = str(Path(os.environ.get("NSBT_BV_PY", "~/venvs/nsbt-bv/bin/python")).expanduser())


def visited_cells(run_dir: Path | None, routes: Routes) -> dict:
    """The cells the aircraft's fused position passed through (fresh fixes, airborne), consecutive repeats removed."""
    f = run_dir / "flight.jsonl" if run_dir else None
    if not f or not f.exists():
        return {"note": "no flight.jsonl"}
    cells, heights, n = [], [], 0
    for line in f.open():
        if '"ekf"' not in line and '"state"' not in line:
            continue
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("kind") == "state" and isinstance(r.get("h"), (int, float)):
            heights.append(r["h"])
        if r.get("kind") == "ekf" and (r.get("z") or 0) > 1.0 and (r.get("sec_since_fix") or 9) < 1.0:
            n += 1
            c = list(routes.cell_of(r["x"], r["y"]))
            if not cells or cells[-1] != c:
                cells.append(c)
    return {"cells_visited": cells, "ekf_rows_used": n, "max_height_m": round(max(heights) / 100, 2) if heights else None}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--example", type=Path, default=ROOT / "examples" / "room1_nn1")
    ap.add_argument("--dry", action="store_true", help="the host's dry run: no arm, no take-off")
    ap.add_argument("--wait", type=float, default=float(os.environ.get("NEO2_WAIT_S", "20")), metavar="S",
                    help="seconds after power-on before take-off (default 20; the runbook's rule is 170)")
    ap.add_argument("--max-s", type=float, default=90.0)
    a = ap.parse_args(argv)
    day = dt.date.today().isoformat()
    logs = HV.LOGS / day
    logs.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%H%M%S")
    rec_file = logs / f"nsbt-{stamp}.json"
    runs = Path(f"~/tello-runs/nsbt-{day}").expanduser()
    rec = {"day": day, "example": a.example.name, "mode": "dry" if a.dry else "armed", "runs": str(runs)}
    joined = False
    try:
        HV.step("Checks")
        for p in (FLIGHT_PY, BV_PY, str(HV.REPO / "tools/neo2_closedloop.py")):
            if not Path(p).exists():
                raise HV.Stop(f"missing on this box: {p}")
        busy = HV.flight_host_running()
        if busy:
            raise HV.Stop("a flight host is already running on this box: " + "; ".join(b[:120] for b in busy))
        print("   nobody else is flying from this box")

        HV.step("The tree, on the flight box: BehaVerify generates it and runs it from every start cell")
        table = logs / f"nsbt-{stamp}-routes.json"
        rc, out = HV.sh(f"{BV_PY} {ROOT / 'examples/route_table.py'} {a.example} --out {table}", timeout=300)
        print(f"   | {out.splitlines()[0] if out else ''}")
        if rc != 0:
            raise HV.Stop(f"the route table could not be made on this box:\n{out[-800:]}")
        made, committed = json.loads(table.read_text()), json.loads((a.example / "routes.json").read_text())
        if made["routes"] != committed["routes"] or made["provenance"] != committed["provenance"]:
            raise HV.Stop("the tree run on this box does not give the committed routes.json (the tree, a network or "
                          "the grid changed): regenerate the table, re-prove and re-simulate before flying")
        routes = Routes(a.example)
        start = routes.ex["flight"]["start_cell"]
        r0 = routes.d["routes"][f"{start[0]},{start[1]}"]
        rec["tree"] = committed["provenance"]
        print(f"   the table equals the committed one (tree {committed['provenance']['tree_sha256'][:12]}); from the "
              f"pad {tuple(start)}: {' '.join(r0['acts'])} -> {tuple(r0['cells'][-1])}")

        print(f"\n   Aircraft on the Neo 2 pad (cell {tuple(start)}): 28 cm from floor tag 60 toward wall tag 15, nose "
              "at wall tag 36 (LAB_HOWTO.md section 5). Fresh pack, nothing within 1.5 m of the route, operator ready "
              "to catch.")
        ans = HV.ask("Power-on time? Enter if it was switched on just now, or HH:MM:")
        now = dt.datetime.now()
        power_on = now if not ans else dt.datetime.combine(now.date(), dt.datetime.strptime(ans, "%H:%M").time())
        rec["power_on"] = power_on.isoformat(timespec="seconds")

        HV.wifi_up()
        joined = True

        while (left := a.wait - (dt.datetime.now() - power_on).total_seconds()) > 0:
            print(f"   .. {left:.0f} s until {a.wait:g} s after power-on", flush=True)
            time.sleep(min(15, left))

        cmd = [FLIGHT_PY, "-B", str(HERE / "nsbt_flight.py"), "host", str(a.example), "--max-s", str(a.max_s),
               "--run-dir", str(runs)]
        if a.dry:
            HV.step("DRY RUN: the host connects, localizes and rehearses the route; the motors never start")
        else:
            HV.step(f"THE FLIGHT: the tree's route from the cell of the first fix, at {routes.ex['flight']['alt_m']} m")
            print("   TO LAND SOONER: ONE Ctrl-C here, or a hand underneath it.")
            if HV.ask("Clear around the route and ready to catch? Type nsbt to take off (anything else cancels):") != "nsbt":
                raise HV.Stop("cancelled: nothing flew.")
            cmd.append("--arm")
        t0 = dt.datetime.now()
        rc, out = HV.stream(cmd, logs / f"nsbt-{'dry' if a.dry else 'flight'}-{t0:%H%M%S}.txt")
        run_dir = HV.run_dir_of(out)
        rec["run"] = {"start": t0.isoformat(timespec="seconds"), "rc": rc, "run_dir": str(run_dir),
                      **visited_cells(run_dir, routes)}
        return 0 if rc == 0 else 2
    except HV.Stop as e:
        print(f"\nSTOPPED: {e}", flush=True)
        rec["stopped"] = str(e)
        return 2
    finally:
        if joined:
            HV.wifi_down()
        rec_file.write_text(json.dumps(rec, indent=1) + "\n")
        if "run" in rec:
            r = rec["run"]
            print(f"\n{'DRY RUN' if a.dry else 'FLOWN'}: exit {r['rc']}, max height {r.get('max_height_m')} m; "
                  f"cells visited {r.get('cells_visited')}")
            print(f"run: {r.get('run_dir')}")
        print(f"notes: {rec_file}")


if __name__ == "__main__":
    sys.exit(main())
