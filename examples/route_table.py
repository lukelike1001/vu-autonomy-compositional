#!/usr/bin/env python3
"""Run an example's BehaVerify-generated tree from EVERY start cell and write the routes it chooses: ``routes.json``.

This is how the tree reaches the aircraft (``hardware/nsbt_flight.py``). The tree and its networks are deterministic,
so the route from a cell is fixed; the table holds the tree's own decisions, tick by tick, for every start, and the
flight host looks up the cell the aircraft is actually in when it gets its first position fix. Nothing is computed
in the air. Each route lists the cells (x, y), the tree's action on each tick, and each cell's centre in the lab's
site frame (E, N metres).

The table records what produced it: the tree's and every network's sha256, the BehaVerify version and the grid.

    ~/venvs/bv/bin/python examples/route_table.py examples/room1_nn1          # -> examples/room1_nn1/routes.json
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import re
import sys
from importlib import metadata
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nsbt_common import ROOT, GeneratedTree, load_example, load_lab_grid  # noqa: E402


def sha(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("example", type=Path)
    ap.add_argument("--out", type=Path, default=None, help="default: <example>/routes.json")
    a = ap.parse_args()
    ex, lab = load_example(a.example), load_lab_grid()
    conv = lab.raw["conventions"]
    E0, N0 = conv["sw_corner_EN_m"]
    cell_m = lab.raw["params"]["cell_m"]
    centre = lambda c: [round(E0 + (c[0] + 0.5) * cell_m, 4), round(N0 + (c[1] + 0.5) * cell_m, 4)]
    tree_text = ex["tree_path"].read_text()
    nets = {Path(src).name: sha(ex["tree_path"].parent / src)
            for src in re.findall(r"source \{\s*'([^']+)'", tree_text)}
    starts = sorted(ex["start_cells"], key=lambda c: (c[1], c[0]))
    g = GeneratedTree(ex["tree_path"])
    routes = {}
    try:
        for s in starts:
            cells, acts = g.route(s)
            bad = [c for c in cells if c not in ex["safe_cells"]]
            if bad or cells[-1] != ex["goal"]:
                raise SystemExit(f"the tree's route from {s} leaves the safe set at {bad[:1]} or ends at {cells[-1]}, "
                                 f"not {ex['goal']}: no table is written")
            routes[f"{s[0]},{s[1]}"] = {"cells": [list(c) for c in cells], "acts": acts,
                                        "en": [centre(c) for c in cells]}
    finally:
        g.close()
    out = a.out or ex["dir"] / "routes.json"
    out.write_text(json.dumps({
        "schema": "nsbt-routes/1",
        "example": ex["dir"].name, "tree": ex["tree"], "goal": list(ex["goal"]),
        "made": dt.datetime.now().isoformat(timespec="seconds"),
        "provenance": {"tree_sha256": sha(ex["tree_path"]), "networks_sha256": nets,
                       "lab_grid_sha256": sha(ROOT / "grid" / "lab_grid.json"),
                       "behaverify": metadata.version("behaverify")},
        "grid": {"cell_m": cell_m, "sw_corner_EN_m": [E0, N0], "W": lab.W, "H": lab.H,
                 "cell": "(x, y): x east, y north; centre E = sw_E + (x + 0.5) * cell_m, N likewise"},
        "safe_cells": sorted(list(c) for c in ex["safe_cells"]),
        "routes": routes}, indent=1) + "\n")
    lengths = [len(r["cells"]) - 1 for r in routes.values()]
    print(f"{out}: {len(routes)} start cells, every route stays in {ex['safe_regions']} and ends at {ex['goal']} "
          f"(moves: {min(lengths)} to {max(lengths)})")
    if "flight" in ex:
        s = ex["flight"]["start_cell"]
        r = routes[f"{s[0]},{s[1]}"]
        print(f"  from the flight start {tuple(s)}: {len(r['cells']) - 1} moves, "
              + " ".join(f"{a}" for a in r["acts"]))


if __name__ == "__main__":
    main()
