# 2026-10-14 Room 1 NSBT Flight Report

**tl;dr:** This worked. A BehaVerify-verified NSBT flew the Neo 2 to its goal: one node, one network, a route of
`So So So` from the cell of the drone's first in-air fix to (7, 12), all 4 waypoints, `COMPLETED`, landed. The tree's
decisions were computed on the ground before take-off, not ticked live. Nothing independent confirms where the drone
really was.

# 1. What flew

The tree `examples/room1_nn1/room1_nn1.tree`, one node: `act := nn1_fly(x, y)`. One move per tick (west, east, north
or south), fixed heading, 1.55 m. NN1-fly is overfitted A* over Room 1's **flyable** cells: those whose centre is 0.60 m
from surveyed structure, and whose connecting legs are too. That is the Neo 2 host's own pre-arm clearance test, run
with its own code and cloud (`grid/flyable.py`). The test leaves 22 cells, with the goal (7, 12), the flyable cell
nearest the doorway.

The plain grid had to go: the host's test passed 0 of its 69 Room 1 routes.

| property, nuXmv | result |
|---|---|
| `INVARSPEC`: always on a flyable cell | **true** |
| `CTLSPEC AF AG (x, y) = (7, 12)`: from every flyable cell, eventually at the goal and there forever | **true** |

How the tree reached the drone:

1. BehaVerify generated the tree on the flight box and ran it from all 22 start cells. The route table matched the
   committed one.
2. In the air, the drone's first tag fix chose the route.
3. The lab's closed-loop host (`neo2_closedloop.py live --mode square`, unchanged) flew it, with only its pattern
   functions replaced by that route.

# 2. The flight

Run `neo2-hardware-20261014T202653Z-3524590`, flown from Taylor's Alienware. Battery 91 %, launched 391 s after
power-on.

| | |
|---|---|
| climb | 1.5 m in 6.0 s, open loop |
| first fix | E 1.30, N 0.81 → cell **(7, 15)**. The pad is at E 1.47, N 0.64, on the line between cells 7 and 8 |
| the tree's decision for (7, 15) | **`So So So XX`** → (7, 12) |
| checks in the air | ≥ 3 tags in view at the flown heading (−18°); route 0.85 m from structure at its tightest (need 0.60) |
| result | **4/4 waypoints, `COMPLETED`**, 63.1 s closed loop, landed |
| airborne | 107.5 s; max height 1.6 m |
| tags lost | 4 times, re-found each time; 360 of 629 control ticks (57 %) spent searching |

# 3. What the track says

The fused position, cell by cell, while airborne with a fresh fix:

| t (s) | E | N | cell |
|---|---|---|---|
| 44.2 | 1.13 | 0.74 | (7, 15) |
| 83.5 | 1.00 | −0.69 | (7, 13) |
| 86.1 | 1.09 | −0.43 | (7, 14) |
| 87.8 | 1.29 | **+0.23** | (7, 15) |
| 97.0 | 1.05 | −1.15 | (7, 12) |
| 100.0 | 1.28 | **−0.44** | (7, 14) |
| 105.2 | 0.90 | −1.10 | (7, 12), final |

- **The northward steps are estimate jumps, not flight.** 0.66 m in 1.7 s and 0.71 m in 3 s, backwards on a route that
  only goes south, right after tags were re-found. The trend underneath is steadily south.
- **There was no fix for 39 s** after the first waypoint (t 44 to 83).

# 4. What this does not say

- **Where the drone actually was.** The ceiling cameras were not recording, so every position above is the drone's own
  tag estimate, with `--no-calib` (nominal camera intrinsics, as the lab's flights use).
- **That the tree ticked live.** Its ticks ran on the ground, against BehaVerify's own one-cell-per-tick model of the
  drone. In the air it was consulted once, at the first fix. That is equivalent to live ticking only if the drone
  always lands in the next cell, and it never saw the jumps above.
- **That the proofs hold for the room.** They are about the grid, the network and the tree. The flyable region makes
  the host's clearance check agree with them; it does not make the survey cloud the room.
- **Earlier attempts.** One took off 1.6 m from the pad and aborted on tag loss after the first move. One was refused
  on the ground (`NO RC STATE`, a fresh pack 20 s after power-on). Neither is recorded here.

# 5. Next

Live ticking: each tick, read the drone's cell, tick the generated tree once, fly one cell, settle, repeat. Record the
ceiling cameras, so the world model's camera truth can check the flown cells independently.

# 6. Reproduce

On the flight box, the Neo 2 on the pad (28 cm from floor tag 60 toward wall tag 15, nose at wall tag 36):

```
cd ~/vu-autonomy-compositional
python3 hardware/neo2_nsbt.py --wait 170      # type nsbt; --wait 170 on a pack's first flight
```

| | version |
|---|---|
| this repo | `c598c18` plus uncommitted changes (the `examples/` restructure, the flyable region, `hardware/`) |
| host | `ttj/vu-autonomy-neo2` `d94ac3b`, core `90f7b20`, venv `~/venvs/neo2-flight` |
| BehaVerify | `verivital/behaverify` `a141baf9b` (laptop `~/venvs/bv`, flight box `~/venvs/nsbt-bv`) |
| nuXmv | 2.1.0 |
| sha256 (first 12) | tree `d0cef553c9de`, `room1_fly.onnx` `4e157c7fc0f0`, `room1_flyable.json` `6bd938fdce50`, `routes.json` `2830d1844b38`, `nsbt_flight.py` `3629372e8bf5`, `neo2_nsbt.py` `16f124d689f9` |

The run (145 MB: video, telemetry, `flight.jsonl`) is on the flight box under `~/tello-runs/nsbt-2026-10-14/`, not
on rundb.
