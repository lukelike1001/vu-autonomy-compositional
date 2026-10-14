# Room 1, one node: the first NSBT to fly

The simplest NSBT that shows a BehaVerify tree flying a drone. The tree is one node:

```
Act    reads the drone's cell: x_cur, y_cur := x_d, y_d;   act := nn1_fly(x_cur, y_cur)
```

It runs once per tick and moves the drone one cell west, east, north or south. It never turns: the host flies a fixed
heading. NN1-fly (`../../networks/room1_fly.onnx`) is overfitted A* over Room 1's **flyable** cells, toward the
flyable cell nearest the doorway, (7, 12), with `XX` (stay) there. The doorway and Room 2 are never entered.

## Why "flyable" and not just "free"

On 2026-10-14 the Neo 2 host's own pre-arm clearance test, run on the routes of the first version of this tree (all of
Room 1, goal (9, 11)), passed **0 of 69**. The pad's route came 0.10 m from surveyed structure, and the goal cell itself
had 0.22 m. The grid only asked a cell to be free, while the host flies a 0.60 m margin (the aircraft plus its hover
wander), so every one of those flights would have been refused in the air. (Those numbers were for cell (5, 14), the
map origin, which an early draft took for the pad. The Neo 2 pad is at E 1.47, N 0.64, in cell (8, 15).)

`grid/flyable.py` therefore applies the host's test, with the host's code and cloud, on the flight box. A cell is
flyable when its centre is 0.60 m from structure at 1.55 m, and a move is flyable when the whole leg between two
centres is. That leaves **22 cells reachable from the Neo 2 pad, cell (8, 15)**. The row north of the doorway is too tight everywhere, so
the goal is (7, 12). The tree is trained, proved and flown on exactly that region, so its safety proof and the host's
check say the same thing.

## Results

| check | result |
|---|---|
| NN1-fly, every flyable cell | 22/22 labels; minimum score margin 1.96; closed loop 22/22 stop at (7, 12), longest 8 moves |
| generated tree, every start (`examples/simulate.py`) | 22/22 starts: always flyable, all settle at (7, 12) |
| nuXmv `INVARSPEC` | **true**: always on a flyable cell |
| nuXmv `CTLSPEC AF AG (x, y) = (7, 12)` | **true**: from every flyable cell, eventually at the goal and there forever |
| the host's clearance test on every route (its code, its cloud, 1.55 m) | 22/22 clear 0.60 m; the Neo 2 pad's route, from (8, 15), 0.85 m at its tightest |
| route table made by BehaVerify on the flight box | identical to `routes.json` |
| the lab's simulator, host's `run_closed_loop`, every start (`hardware/nsbt_flight.py sitl`) | 22/22 `completed`; final within 0.14 m of the goal; 100 % of the track on flyable cells; ≥ 0.61 m from structure; heading held to about 1° |

From the Neo 2 pad (E 1.47, N 0.64, by floor tag 60; cell (8, 15)) the tree's route is `We So So So XX`: one cell west,
three south, to (7, 12). That is E 1.77 N 0.52 to E 1.12 N −1.43, cell centres. In the simulator from that cell:
`completed`, 0.02 m from the goal, at least 0.85 m from structure.

**Not shown here:** the simulator's pose is the truth plus noise, so it says nothing about the tag localizer, and its
speeds are declared, not measured. None of this is a flight.

## Files

| file | what |
|---|---|
| `example.json` | the tree, its goal, its start and safe regions, the flight's start cell, altitude and margin |
| `generate_tree.py` | writes `room1_nn1.tree` from `grid/room1_flyable.json` and the network |
| `room1_nn1.tree` | the NSBT |
| `smv/nuxmv/` | the nuXmv model and proof log |
| `routes.json` | the tree's route from every flyable cell (`examples/route_table.py`), what the aircraft flies |
| `simulation_results.json` | the every-start run of the generated tree |

To fly it, see `hardware/README.md`.
