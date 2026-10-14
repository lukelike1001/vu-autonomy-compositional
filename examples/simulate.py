#!/usr/bin/env python3
"""Run an example's BehaVerify-generated tree from EVERY start cell, and check its safety and liveness.

For ``examples/<name>/example.json``:

1. ``behaverify python <tree>`` generates the executable tree (py_trees + onnxruntime) in a temporary folder;
2. for every cell of the start set, the generated environment is initialised, ``x_d, y_d`` are set to that cell, and
   the tree is ticked ``--ticks`` times with the generated runner's own ``full_tick``;
3. a run fails if it ever stands outside the safe set, never reaches the goal, or is not at the goal at the end
   (and, where example.json names ``must_pass``, if it never passes that region).

This executes the generated code; it is not the nuXmv proof. It is exhaustive over start cells because the tree and
the networks are deterministic once the start is chosen. Writes ``simulation_results.json`` into the example.

    ~/venvs/bv/bin/python examples/simulate.py examples/room1_nn1      # exit 0: the tree passes from every start
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nsbt_common import load_example, load_lab_grid, simulate  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("example", type=Path, help="an example directory, e.g. examples/room1_nn1")
    ap.add_argument("--ticks", type=int, default=60, help="ticks per run (the longest A* route is 22 moves)")
    a = ap.parse_args()
    ex = load_example(a.example)
    r = simulate(ex, load_lab_grid(), a.ticks)
    ok = r["failures"] == 0
    print(f"{r['tree']}: {r['starts']} starts, {r['failures']} failing runs ({r['unsafe_runs']} unsafe, "
          f"{r['no_goal_runs']} never settle at the goal {tuple(r['goal'])}); longest to goal "
          f"{r['longest_to_goal']} ticks; e.g. {r['first_failures'][:1]}")
    (ex["dir"] / "simulation_results.json").write_text(json.dumps({"passed": ok, **r}, indent=1) + "\n")
    print("PASS: the tree holds from every start" if ok else "FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
