# vu-autonomy-compositional

Compositional neuro-symbolic behavior trees (NSBTs) for the VU autonomy lab: abstract the lab into a grid world of
several rooms, give each room its own neural network, compose them with a behavior tree, and verify safety and
liveness with [BehaVerify](https://github.com/verivital/behaverify) and nuXmv before trying the same tree on hardware.

A result in this repository is a statement about a grid, a set of networks and a tree, not about the room or a flight.

## Pipeline

| step | folder | script | output | venv |
|---|---|---|---|---|
| 1 | [`grid/`](grid/) | `discretize_lab.py` | the 0.65 m two-room grid world, `lab_grid.json` | `~/venvs/vu-flight` |
| 2 | [`networks/`](networks/) | `train_astar_networks.py` | `room1.onnx`, `door.onnx`, `room2.onnx` (overfitted A*) | `~/venvs/nsbt` |
| 3 | [`tree/`](tree/) | `generate_tree.py`, `simulate_all_starts.py` | `two_rooms_nsbt.tree`, every-start run | `~/venvs/bv` |
| 4 | [`smv/`](smv/) | BehaVerify + nuXmv | the nuXmv models and proof logs | `~/venvs/bv` |

`grid/lab_grid.py` is the one reader of `lab_grid.json`; steps 2 and 3 import it.

```
~/venvs/vu-flight/bin/python grid/discretize_lab.py           # about a minute; reads ../vu-autonomy (below)
~/venvs/nsbt/bin/python      networks/train_astar_networks.py # about 10 s; exit 0 = every check passed
python3                      tree/generate_tree.py            # writes the tree
~/venvs/bv/bin/python        tree/simulate_all_starts.py      # about 5 s; exit 0 = the tree passes from every start
~/venvs/bv/bin/behaverify nuxmv tree/two_rooms_nsbt.tree smv/ --generate --invar --ctl --nuxmv_path ./nuXmv
```

## Environments

| venv | contents |
|---|---|
| `~/venvs/vu-flight` | Python 3.12, `numpy`, `opencv-contrib-python`, `matplotlib` |
| `~/venvs/nsbt` | Python 3.12, CPU `torch`, `onnx`, `onnxruntime`, `numpy`, `matplotlib` |
| `~/venvs/bv` | Python 3.12, BehaVerify installed editable from a clone of `verivital/behaverify`, plus `onnx`, `onnxruntime` |

nuXmv 2.1.0 comes from FBK under an academic, non-commercial licence that forbids redistribution. `./install_nuxmv.sh`
downloads it to `./nuXmv` in the repo root (running it means accepting the licence at https://nuxmv.fbk.eu/). The
binary is git-ignored: never commit it or any part of its distribution.

## Inputs from other repositories

- The survey grid `footprint4d.npz` from `ttj/vu-autonomy` (`scripts/flights/slices/v6.2-transit-z155-w030-v020/`),
  read from a sibling clone at `../vu-autonomy`. `grid/lab_grid.json` records its repository, path, sha256 and commit.
- The NSBT follows the NeuS 2025 grid world in `verivital/behaverify` (`REPRODUCIBILITY/2025_NEUS/examples/grid_world/`).
