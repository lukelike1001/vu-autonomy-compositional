#!/usr/bin/env python3
"""Run the BehaVerify-generated python of each tree from EVERY start cell, and check safety and liveness.

For the tree written by ``generate_tree.py``:

1. ``behaverify python <tree>`` generates the executable tree (py_trees + onnxruntime) in a temporary folder;
2. for every cell with ``cellmap`` 1, 2 or 3 (Room 1, the doorway, Room 2), the generated environment is
   initialised, ``x_d, y_d`` are set to that cell, and the tree is ticked with the generated runner's own
   ``full_tick`` (pre-tick update, tick, delayed actions, post-tick update);
3. after every tick: SAFETY, the drone is on a region cell; LIVENESS, the run reaches NN2's goal within
   ``--ticks`` ticks and is still there at the end; DOORWAY, a run that starts in Room 1 passes the doorway cell.

This executes the generated code; it is not the nuXmv proof (``two_rooms_nsbt.tree``'s INVARSPEC and CTLSPEC). It is
exhaustive over start cells because the tree and the networks are deterministic once the start is chosen.

Writes ``simulation_results.json`` next to this script.

    ~/venvs/bv/bin/python tree/simulate_all_starts.py      # about 5 s; exit 0: the tree passes
"""
from __future__ import annotations

import argparse
import importlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
BEHAVERIFY = Path(sys.executable).parent / "behaverify"
sys.path.insert(0, str(HERE.parent / "grid"))
sys.path.insert(0, str(HERE))
from generate_tree import TREE_FILE, nn2_goal  # noqa: E402
from lab_grid import DOOR, ROOM1, ROOM2, load_lab_grid  # noqa: E402

IN_REGIONS = (ROOM1, DOOR, ROOM2)


def generate(tree: Path, out: Path):
    r = subprocess.run([str(BEHAVERIFY), "python", str(tree), str(out), "--overwrite"], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"behaverify python failed on {tree.name}:\n{r.stderr[-2000:]}")


def run_tree(tree: Path, ticks: int, lab, goal):
    with tempfile.TemporaryDirectory() as tmp:
        generate(tree, Path(tmp))
        pydir = Path(tmp) / "python"
        sys.path.insert(0, str(pydir))
        stem = tree.stem
        try:
            mod = importlib.import_module(stem)
            env_mod = importlib.import_module(f"{stem}_environment")
            runner = importlib.import_module(f"{stem}_runner")
            rand_mod = importlib.import_module("serene_randomizer")
            import py_trees

            def fresh():
                py_trees.blackboard.Blackboard.clear()
                rnd = rand_mod.serene_randomizer()
                bb = mod.create_blackboard(rnd)
                env = getattr(env_mod, f"{stem}_environment")(bb)
                rnd.set_blackboard_and_environment(bb, env)
                mod.initialize_blackboard(bb)
                env.initialize_environment()
                tree_ = py_trees.trees.BehaviourTree(mod.create_tree(env))
                return bb, env, tree_

            _, env0, _ = fresh()
            if tuple(env0.cellmap) != lab.cellmap:
                raise SystemExit(f"{tree.name}: the tree's cellmap differs from grid/lab_grid.json; regenerate the trees")
            (gx, gy), (dx, dy) = goal, lab.door
            region = lab.code
            starts = [(x, y) for y in range(lab.H) for x in range(lab.W) if lab.code(x, y) in IN_REGIONS]
            fails, longest = [], 0
            for (sx, sy) in starts:
                bb, env, tr = fresh()
                env.x_d, env.y_d = sx, sy
                path, reached, unsafe, at_door = [(sx, sy)], None, None, (sx, sy) == (dx, dy)
                for t in range(1, ticks + 1):
                    runner.full_tick(tr, env)
                    c = (env.x_d, env.y_d)
                    path.append(c)
                    if region(*c) not in IN_REGIONS and unsafe is None:
                        unsafe = (t, c, bb.region, bb.act)
                    if c == (dx, dy):
                        at_door = True
                    if c == (gx, gy) and reached is None:
                        reached = t
                stayed = path[-1] == (gx, gy)
                door_ok = at_door or region(sx, sy) != ROOM1
                if reached is not None:
                    longest = max(longest, reached)
                if unsafe or reached is None or not stayed or not door_ok:
                    fails.append({"start": [sx, sy], "unsafe_at": unsafe, "reached_goal_at": reached,
                                  "at_goal_at_end": stayed, "passed_door": at_door, "end": list(path[-1])})
            return {"starts": len(starts), "ticks": ticks, "failures": len(fails), "longest_to_goal": longest,
                    "unsafe_runs": sum(1 for f in fails if f["unsafe_at"]),
                    "no_goal_runs": sum(1 for f in fails if f["reached_goal_at"] is None or not f["at_goal_at_end"]),
                    "first_failures": fails[:3]}
        finally:
            sys.path.remove(str(pydir))
            for m in [m for m in sys.modules if m.startswith(stem) or m in ("serene_randomizer", "Assign_file", "Act_file")]:
                del sys.modules[m]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ticks", type=int, default=60, help="ticks per run (the longest A* route is 22 moves)")
    a = ap.parse_args()
    lab, goal = load_lab_grid(), nn2_goal()
    r = run_tree(TREE_FILE, a.ticks, lab, goal)
    print(f"{TREE_FILE.name}: {r['starts']} starts, {r['failures']} failing runs "
          f"({r['unsafe_runs']} unsafe, {r['no_goal_runs']} never settle at the goal); "
          f"longest to goal {r['longest_to_goal']} ticks; e.g. {r['first_failures'][:1]}")
    ok = r["failures"] == 0
    (HERE / "simulation_results.json").write_text(json.dumps({"tree": TREE_FILE.name, "passed": ok, **r}, indent=1) + "\n")
    print("PASS: the tree holds from every start" if ok else "FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
