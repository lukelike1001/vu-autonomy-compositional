#!/usr/bin/env python3
"""Write the one-node Room 1 NSBT as a BehaVerify ``.tree`` file.

    Act    reads the drone's cell: x_cur, y_cur := x_d, y_d;  act := nn1_fly(x_cur, y_cur)

That is the whole tree: one action, ticked once per step. NN1-fly (``networks/room1_fly.onnx``) is A* over Room 1's
FLYABLE cells (``grid/room1_flyable.json``: the cells and legs the Neo 2 host's own 0.60 m clearance test accepts at
1.55 m) toward the flyable cell nearest the doorway, with XX (stay) there. The environment moves the drone one cell
in the direction ``act``, as in the NeuS 2025 grid world.

The drone starts on ANY flyable cell. Specifications:

* ``INVARSPEC`` safety: the drone is always on a flyable cell (``flyable = 1``). That rules out every obstacle, the
  doorway and Room 2, and, because flyable.py refuses a region with an unflyable leg between adjacent cells, every
  one-cell move it makes is a leg the host's clearance test accepts.
* ``CTLSPEC`` liveness: ``AF AG (x_d, y_d) = GOAL``: from every start, the drone eventually reaches the goal and
  stays there forever.

    python3 examples/room1_nn1/generate_tree.py        # writes room1_nn1.tree next to this script
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from nsbt_common import (ACTIONS, ENV_UPDATE, LabGrid, header, load_example, load_lab_grid,  # noqa: E402
                         member_decl, net_sources, neural_decl, start_decl)

FLYABLE = "(index, flyable, (add, (mult, y_d, W), x_d))"


def tree_text(lab: LabGrid, goal, cells, nets: dict) -> str:
    gx, gy = goal
    return header("Room 1, one-node NSBT (NN1-fly).", "examples/room1_nn1/generate_tree.py",
                  "grid/room1_flyable.json and\n   networks/room1_fly.onnx") + f"""configuration {{
    neural
}}
enumerations {{
    {', '.join(repr(a) for a in ACTIONS)}
}}
constants {{
    W := {lab.W}, H := {lab.H}, GOAL_X := {gx}, GOAL_Y := {gy}
}} end_constants

variables {{
    variable {{ bl act VAR {{{', '.join(repr(a) for a in ACTIONS)}}} assign{{result{{'XX'}}}}}}
    #{{ the cell the tree believes it is in: copied from the environment by Act on every tick }}#
    variable {{ bl x_cur VAR [0, (sub, W, 1)] assign{{result{{0}}}}}}
    variable {{ bl y_cur VAR [0, (sub, H, 1)] assign{{result{{0}}}}}}
{member_decl(lab, "flyable", cells, "flyable[y * W + x]: 1 on Room 1's flyable cells (grid/room1_flyable.json), 0 elsewhere")}
{start_decl(lab, cells, "the start: any flyable cell")}
{neural_decl(nets)}
}} end_variables

{ENV_UPDATE}

checks {{
}} end_checks

environment_checks {{
}} end_environment_checks

actions {{
    action {{
\tAct
\targuments {{}}
\tlocal_variables {{}} end_local_variables
\tread_variables {{nn1_fly}} end_read_variables
\twrite_variables {{act, x_cur, y_cur}} end_write_variables
\tinitial_values {{}} end_initial_values
\tupdate {{
\t    read_environment {{
\t\tread_cell
\t\tcondition {{True}}
\t\tvariable_statement {{
\t\t    x_cur
\t\t    assign {{result {{x_d}}}}
\t\t}}
\t\tvariable_statement {{
\t\t    y_cur
\t\t    assign {{result {{y_d}}}}
\t\t}}
\t    }}
\t    variable_statement {{
\t\tact
\t\tassign {{result {{nn1_fly}}}}
\t    }}
\t    return_statement {{
\t\tresult {{success}} end_result
\t    }} end_return_statement
\t}} end_update
    }} end_action
}} end_actions

sub_trees {{}} end_sub_trees

tree {{
    Act {{}}
}} end_tree

tick_prerequisite {{ (True) }} end_tick_prerequisite

specifications {{
    #{{ SAFETY: always on a flyable cell: never an obstacle, never the doorway or Room 2, and every one-cell move a
       leg the Neo 2 host's 0.60 m clearance test accepts. }}#
    INVARSPEC {{
\t(eq, {FLYABLE}, 1)
    }}
    #{{ LIVENESS: from every start, eventually at (GOAL_X, GOAL_Y), the flyable cell nearest the doorway, and there
       forever. }}#
    CTLSPEC {{
\t(always_finally, (always_globally, (and, (eq, x_d, GOAL_X), (eq, y_d, GOAL_Y))))
    }}
}} end_specifications
"""


def main():
    lab = load_lab_grid()
    ex = load_example(HERE, lab)
    nets = {"nn1_fly": net_sources(HERE, ("room1_fly",))["room1_fly"]}        # the tree calls it nn1_fly
    ex["tree_path"].write_text(tree_text(lab, ex["goal"], ex["start_cells"], nets))
    print(f"wrote {ex['tree_path'].relative_to(HERE.parent.parent)}  (goal {ex['goal']}, "
          f"{len(ex['start_cells'])} flyable start cells)")


if __name__ == "__main__":
    main()
