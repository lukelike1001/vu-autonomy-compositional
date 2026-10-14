#!/usr/bin/env python3
"""Write the two-room NSBT as a BehaVerify ``.tree`` file, from the grid and the three networks.

The tree (after the NeuS 2025 grid world, ``REPRODUCIBILITY/2025_NEUS/examples/grid_world/template.tree``):

    Root (sequence)
    ├── Assign   reads the drone's cell from the environment: x_cur, y_cur := x_d, y_d and
    │            region := 'R1' if y_d > DOOR_Y, 'D' if y_d == DOOR_Y, else 'R2'
    └── Act      act := room1(x_cur, y_cur) if region = 'R1', door(...) if 'D', room2(...) if 'R2'

``cellmap`` is the whole-lab grid as an array indexed by ``y * W + x``: 0 obstacle, 1 Room 1, 2 doorway, 3 Room 2,
4 free but unreachable. The drone starts on any cell with ``cellmap`` 1, 2 or 3.

Specifications:

* ``INVARSPEC`` safety: the drone never stands on an obstacle cell (``cellmap != 0``), from any of the 116 starts.
* ``CTLSPEC`` liveness: ``AG ((x_d, y_d) = START -> AF (x_d, y_d) = GOAL)``: from the start cell (default (16, 13),
  the middle of Room 1's rightmost column) the drone eventually reaches NN2's goal.

The y-test in ``Assign`` is a shortcut that is exact for THIS grid only (the doorway is the only free cell in its
row); ``check_split`` refuses to write a tree if it ever disagrees with the regions in ``lab_grid.json``.

    python3 examples/two_rooms/generate_tree.py        # writes two_rooms_nsbt.tree next to this script
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from nsbt_common import (ACTIONS, CELL, DOOR, ENV_UPDATE, ROOM1, ROOM2, LabGrid, cellmap_decl, header,  # noqa: E402
                         load_example, load_lab_grid, net_sources, neural_decl, start_decl)


def check_split(lab: LabGrid):
    door_y = lab.door[1]
    for name, code in (("room1", ROOM1), ("door", DOOR), ("room2", ROOM2)):
        for (x, y) in lab.regions[name]:
            by_y = ROOM1 if y > door_y else DOOR if y == door_y else ROOM2
            if by_y != code:
                raise SystemExit(f"the y-split disagrees with the grid at ({x}, {y}): y says {by_y}, grid says {code}")


def tree_text(lab: LabGrid, goal, start, nets: dict) -> str:
    door_x, door_y = lab.door
    gx, gy = goal
    return header("Two-room grid world NSBT.", "examples/two_rooms/generate_tree.py",
                  "grid/lab_grid.json and\n   networks/*.onnx") + f"""configuration {{
    neural
}}
enumerations {{
    {', '.join(repr(a) for a in ACTIONS)}, 'R1', 'D', 'R2'
}}
constants {{
    W := {lab.W}, H := {lab.H}, DOOR_X := {door_x}, DOOR_Y := {door_y}, GOAL_X := {gx}, GOAL_Y := {gy}, START_X := {start[0]}, START_Y := {start[1]}
}} end_constants

variables {{
    variable {{ bl act VAR {{{', '.join(repr(a) for a in ACTIONS)}}} assign{{result{{'XX'}}}}}}
    variable {{ bl region VAR {{'R1', 'D', 'R2'}} assign{{result{{'R1'}}}}}}
    #{{ the cell the tree believes it is in: copied from the environment by Assign on every tick }}#
    variable {{ bl x_cur VAR [0, (sub, W, 1)] assign{{result{{0}}}}}}
    variable {{ bl y_cur VAR [0, (sub, H, 1)] assign{{result{{0}}}}}}
{cellmap_decl(lab)}
{start_decl(lab, lab.regions["room1"] | lab.regions["door"] | lab.regions["room2"],
            "the start: any cell of Room 1, the doorway or Room 2")}
{neural_decl(nets)}
}} end_variables

{ENV_UPDATE}

checks {{
}} end_checks

environment_checks {{
}} end_environment_checks

actions {{
    action {{
\tAssign
\targuments {{}}
\tlocal_variables {{}} end_local_variables
\tread_variables {{}} end_read_variables
\twrite_variables {{region, x_cur, y_cur}} end_write_variables
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
\t\tvariable_statement {{
\t\t    region
\t\t    assign {{
\t\t\tcase {{(gt, y_d, DOOR_Y)}} result {{'R1'}}
\t\t\tcase {{(eq, y_d, DOOR_Y)}} result {{'D'}}
\t\t\tresult {{'R2'}}
\t\t    }}
\t\t}}
\t    }}
\t    return_statement {{
\t\tresult {{success}} end_result
\t    }} end_return_statement
\t}} end_update
    }} end_action
    action {{
\tAct
\targuments {{}}
\tlocal_variables {{}} end_local_variables
\tread_variables {{region, {', '.join(nets)}}} end_read_variables
\twrite_variables {{act}} end_write_variables
\tinitial_values {{}} end_initial_values
\tupdate {{
\t    variable_statement {{
\t\tact
\t\tassign {{
\t\t    case {{(eq, region, 'R1')}} result {{room1}}
\t\t    case {{(eq, region, 'D')}} result {{door}}
\t\t    result {{room2}}
\t\t}}
\t    }}
\t    return_statement {{
\t\tresult {{success}} end_result
\t    }} end_return_statement
\t}} end_update
    }} end_action
}} end_actions

sub_trees {{}} end_sub_trees

tree {{
    composite {{
\tRoot sequence
\tchildren {{
\t    Assign {{}}
\t    Act {{}}
\t}}
    }}
}} end_tree

tick_prerequisite {{ (True) }} end_tick_prerequisite

specifications {{
    #{{ SAFETY: the drone never crashes into an obstacle, whichever network (NN1, the doorway's, NN2) moved it.
       cellmap is 0 exactly on obstacle cells; the grid's border is all obstacle, so leaving the grid is a crash too. }}#
    INVARSPEC {{
\t(neq, {CELL}, 0)
    }}
    #{{ LIVENESS: from the start cell (START_X, START_Y), the rightmost of Room 1, the drone eventually reaches the goal
       (GOAL_X, GOAL_Y), where NN2 was trained to converge. Written AG (start -> AF goal) so that it does not depend on
       how BehaVerify's encoding lines up the initial state with the first tick. }}#
    CTLSPEC {{
\t(always_globally, (implies, (and, (eq, x_d, START_X), (eq, y_d, START_Y)), (always_finally, (and, (eq, x_d, GOAL_X), (eq, y_d, GOAL_Y)))))
    }}
}} end_specifications
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", default="16,13", help="the liveness start cell x,y (default: the middle of Room 1's "
                    "rightmost column, x = 16)")
    a = ap.parse_args()
    ex, lab = load_example(HERE), load_lab_grid()
    check_split(lab)
    start = tuple(int(v) for v in a.start.split(","))
    if lab.code(*start) != ROOM1:
        raise SystemExit(f"the start {start} is not a Room 1 cell")
    nets = net_sources(HERE, ("room1", "door", "room2"))
    ex["tree_path"].write_text(tree_text(lab, ex["goal"], start, nets))
    print(f"wrote {ex['tree_path'].relative_to(HERE.parent.parent)}")
    print(f"  {lab.W} x {lab.H} grid, doorway {lab.door}, start {start}, goal {ex['goal']}, "
          f"{sum(c in (ROOM1, DOOR, ROOM2) for c in lab.cellmap)} start cells for safety")


if __name__ == "__main__":
    main()
