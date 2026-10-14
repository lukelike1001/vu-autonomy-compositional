# vu-autonomy-compositional

Compositional neuro-symbolic behavior trees (NSBTs) for the VU autonomy lab: abstract the lab into a grid world of
several rooms, give each room its own neural network, compose them with a behavior tree, and verify safety and
liveness with [BehaVerify](https://github.com/verivital/behaverify) and nuXmv before flying the same tree on hardware.

A result in this repository is a statement about a grid, a set of networks and a tree, not about the room or a flight.

## Layout

| folder | what | venv |
|---|---|---|
| [`grid/`](grid/) | the lab as a 0.65 m two-room grid (`discretize_lab.py` → `lab_grid.json`); `flyable.py` → `room1_flyable.json`, the Room 1 cells the Neo 2 host's own 0.60 m clearance test accepts | `~/venvs/vu-flight`; `flyable.py` on the flight box |
| [`networks/`](networks/) | the overfitted A* networks (`train_astar_networks.py`): `room1`, `door`, `room2`, and `room1_fly` | `~/venvs/nsbt` |
| [`examples/`](examples/) | one folder per NSBT: its tree, `example.json`, nuXmv proofs (`smv/`) and route table; `simulate.py` and `route_table.py` run any of them | `~/venvs/bv` |
| [`hardware/`](hardware/) | the Neo 2 runners, run on the flight box: `neo2_hover.py` (take off, hover, land), `neo2_nsbt.py` (fly an NSBT), and `a51.sh` to reach the box | flight box |
| [`reports/`](reports/) | dated snapshots of results | |

`grid/lab_grid.py` is the one reader of `lab_grid.json`; `examples/nsbt_common.py` holds what every example shares.

### The examples

| example | tree | proved (nuXmv) | flies |
|---|---|---|---|
| [`two_rooms`](examples/two_rooms/) | `Assign` picks NN1, the doorway's or NN2 by row; `Act` calls it | never on an obstacle; from (16, 13) eventually at (5, 5) | no (crosses the doorway) |
| [`room1_nn1`](examples/room1_nn1/) | one node: `act := nn1_fly(x, y)` | always on a flyable Room 1 cell; from every flyable cell, eventually at (7, 12) and there forever | yes: `hardware/neo2_nsbt.py` |

```
~/venvs/vu-flight/bin/python grid/discretize_lab.py                  # about a minute; reads ../vu-autonomy (below)
~/venvs/nsbt/bin/python      networks/train_astar_networks.py        # about 10 s; exit 0 = every check passed
python3                      examples/room1_nn1/generate_tree.py     # writes the tree
~/venvs/bv/bin/python        examples/simulate.py examples/room1_nn1 # exit 0 = the tree passes from every start
~/venvs/bv/bin/behaverify nuxmv examples/room1_nn1/room1_nn1.tree examples/room1_nn1/smv/ --generate --invar --ctl --nuxmv_path ./nuXmv
~/venvs/bv/bin/python        examples/route_table.py examples/room1_nn1  # the routes the tree flies, for the hardware
```

## Environments

| venv | contents |
|---|---|
| `~/venvs/vu-flight` | Python 3.12, `numpy`, `opencv-contrib-python`, `matplotlib` |
| `~/venvs/nsbt` | Python 3.12, CPU `torch`, `onnx`, `onnxruntime`, `numpy`, `matplotlib` |
| `~/venvs/bv` | Python 3.12, BehaVerify installed editable from a clone of `verivital/behaverify`, plus `onnx`, `onnxruntime` |
| on the flight box: `~/venvs/nsbt-bv` | Python 3.12, BehaVerify `a141baf9b` from GitHub and the same package versions as `~/venvs/bv` |
| on the flight box: `~/venvs/neo2-flight` | the lab's flight environment, used as it is (never changed by this repository) |

nuXmv 2.1.0 comes from FBK under an academic, non-commercial licence that forbids redistribution. `./install_nuxmv.sh`
downloads it to `./nuXmv` in the repo root (running it means accepting the licence at https://nuxmv.fbk.eu/). The
binary is git-ignored: never commit it or any part of its distribution.

## Inputs from other repositories

- The survey grid `footprint4d.npz` from `ttj/vu-autonomy` (`scripts/flights/slices/v6.2-transit-z155-w030-v020/`),
  read from a sibling clone at `../vu-autonomy`. `grid/lab_grid.json` records its repository, path, sha256 and commit.
- The flyable region is computed with `ttj/vu-autonomy-neo2`'s own clearance code and the lab's v6.2 survey cloud, on
  the flight box; `grid/room1_flyable.json` records both.
- The NSBT follows the NeuS 2025 grid world in `verivital/behaverify` (`REPRODUCIBILITY/2025_NEUS/examples/grid_world/`).
