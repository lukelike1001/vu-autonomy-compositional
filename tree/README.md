# Step 3: the two-room NSBT

A neuro-symbolic behavior tree that flies the 0.65 m two-room grid (`../grid/`) with the three A* networks
(`../networks/`), written in BehaVerify's DSL after the NeuS 2025 grid world
(`behaverify/REPRODUCIBILITY/2025_NEUS/examples/grid_world/template.tree`). Every result here is about this tree,
this grid and these networks, not about the room or a flight.

```
Root (sequence)
├── Assign   reads the drone's cell: x_cur, y_cur := x_d, y_d
│            region := 'R1' if y_d > 10, 'D' if y_d == 10, else 'R2'
└── Act      act := room1(x_cur, y_cur) | door(x_cur, y_cur) | room2(x_cur, y_cur), chosen by region
```

The environment then moves the drone one cell in direction `act` (`We`, `Ea`, `No`, `So`; `XX` stays). As in the NeuS
grid world, a move is clamped to the grid's edge and is NOT blocked by obstacles, so a move into an obstacle is visible
to the safety specification.

- **The doorway row is `y = 10`.** The y-test is exact for this grid only, because the doorway `(9, 10)` is the only
  free cell in its row. `generate_tree.py` refuses to write a tree if the split ever disagrees with `lab_grid.json`.
- **The networks read the blackboard copy `x_cur, y_cur`, not `x_d, y_d`.** BehaVerify's python generator reads every
  network input from the blackboard, so a network fed environment variables fails at run time in python. The copy
  is also what the design says: `Assign` knows which cell we are in.
- **Starts:** any of the 116 cells of Room 1, the doorway and Room 2. The two unreachable free cells `(18, 11)` and
  `(19, 11)` are excluded: the y-split would send them to NN1, which was never trained there.

## Specifications (in `two_rooms_nsbt.tree`)

| | natural language | nuXmv |
|---|---|---|
| safety | the drone never crashes into an obstacle, whichever network moved it | `INVARSPEC cellmap[y_d * 21 + x_d] != 0` |
| liveness | from the start `(16, 13)` (the middle of Room 1's rightmost column), the drone eventually reaches NN2's goal `(5, 5)` | `CTLSPEC AG ((x_d = 16 & y_d = 13) -> AF (x_d = 5 & y_d = 5))` |

Safety holds from all 116 starts. The grid's border is all obstacle, so leaving the grid counts as a crash. Liveness is
written `AG (start -> AF goal)` so that it does not depend on how BehaVerify's encoding lines up nuXmv's initial state
with the first tick. `--start x,y` picks another start cell. The goal is read from `../networks/astar_labels.json`, so
it follows NN2 if NN2 is retrained.

**Not yet proved.** BehaVerify translates the tree to a 1,824-line SMV with both specifications (the networks are
stored as tables, the NeuS paper's fastest strategy). With nuXmv installed by `../install_nuxmv.sh`, from the repo root:

```
~/venvs/bv/bin/behaverify nuxmv tree/two_rooms_nsbt.tree smv/ --generate --invar --ctl --nuxmv_path ./nuXmv
```

## Executed from every start (`simulate_all_starts.py`)

`behaverify python` generates the tree into a temporary folder. The harness starts the generated tree on every one of
the 116 cells and ticks it 60 times with the generated runner's own `full_tick`. This is the generated code running,
not the nuXmv proof. Results are in `simulation_results.json`.

All 116 runs stay on Room 1, doorway or Room 2 cells and settle at `(5, 5)`; the longest route takes 22 ticks.

## Files

| file | what |
|---|---|
| `generate_tree.py` | writes the tree from `../grid/lab_grid.json` and `../networks/` (`python3 tree/generate_tree.py [--start x,y]`) |
| `two_rooms_nsbt.tree` | the NSBT |
| `simulate_all_starts.py` | generates the tree's python with BehaVerify and runs it from every start (`~/venvs/bv/bin/python`, about 5 s; exit 0 = the tree passes) |
| `simulation_results.json` | the result above |
