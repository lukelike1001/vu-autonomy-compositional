# 2026-10-12 Two Rooms nuXmv Report

**tl;dr:** This worked. nuXmv proves both properties of the two-room NSBT: the drone never hits an obstacle, and
from the right edge of Room 1 it always reaches the goal in Room 2. Under 1 second. Software only; hardware is next.

# 1. What was checked

The tree `tree/two_rooms_nsbt.tree` on the 0.65 m two-room grid. Every tick, `Assign` reads the drone's cell and
picks a network by row (y > 10: NN1, y = 10: doorway, y < 10: NN2), and `Act` moves one cell where that network says.

| property | plain English | nuXmv | result |
|---|---|---|---|
| safety | never on an obstacle cell, whichever network moved it, from any of the 116 start cells | `INVARSPEC cellmap[y*21 + x] != 0` | **true** |
| liveness | from (16, 13) (right edge of Room 1), eventually at (5, 5) (middle of Room 2) | `CTLSPEC AG (at (16,13) -> AF at (5,5))` | **true** |

Run time 0.99 s, 85 MB. 232 reachable states.

# 2. Why we trust it

- **The liveness proof is not empty.** It is an "if at the start, then ..." statement, so it would also come back true
  if the drone could never be at the start. nuXmv finds a state at (16, 13), so the "if" really happens.
- **It goes through the doorway and stays at the goal.** Two extra checks, both true: from (16, 13) the drone always
  reaches the doorway (9, 10), and once at (5, 5) it stays there forever.
- **The route is the expected one.** 19 moves:
  (16,13) → (16,12) → (15,12) → (14,12) → (13,12) → (13,11) → (12,11) → (11,11) → (10,11) → (9,11) →
  **(9,10) doorway** → (9,9) → (8,9) → (8,8) → (8,7) → (8,6) → (8,5) → (7,5) → (6,5) → (5,5).
  nuXmv's trace and the A* labels give the same route.
- **Running the tree agrees.** BehaVerify's generated python, started from all 116 cells: 0 crashes, all settle at (5, 5).

# 3. What this does not say

- **It is a proof about the grid, not the room.** 0.65 m cells, obstacles rounded up, heights ignored, one cell per
  tick, no drift, perfect position. A real flight has none of that.
- **The margin is thin for the Neo 2.** The drone at a cell centre is 0.325 m from the cell edge. The Neo 2 hover
  drift is about 0.56 m peak to peak. Holding the cell is the hardware's job, not the tree's.
- **Grid-specific shortcuts.** The y-split only works because the doorway is the only free cell in row 10. The
  networks are meaningless outside their own room.

# 4. Next

Fly it. Same start (16, 13), same goal (5, 5), cameras watching. If the real drone stays out of obstacles and reaches
the goal, the grid result holds up. If not, back to the drawing board.

# 5. Reproduce

```
./install_nuxmv.sh
~/venvs/bv/bin/behaverify nuxmv tree/two_rooms_nsbt.tree smv/ --generate --invar --ctl --nuxmv_path ./nuXmv
./nuXmv -source smv/sanity_checks.cmd smv/nuxmv/two_rooms_nsbt.smv      # the checks in section 2
```

Output: `smv/nuxmv/two_rooms_nsbt_output.txt` (the two proofs), `smv/sanity_checks_output.txt` (section 2).

| | version |
|---|---|
| this repo | `2c7b017` |
| nuXmv | 2.1.0 |
| BehaVerify | `verivital/behaverify` `a141baf9b` |
| survey grid | `ttj/vu-autonomy` `08d9e42` |
| sha256 (first 12) | tree `15302d1aaf9e`, smv `5e0ea97ffef0`, `room1.onnx` `30555e2048af`, `door.onnx` `72c014c87478`, `room2.onnx` `6d1349e9c6f6`, `lab_grid.json` `be68f0064184` |
