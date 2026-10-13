#!/usr/bin/env python3
"""Train the three grid-world networks for the two-room NSBT and export them as ONNX.

Each network is an A* policy overfitted on EVERY free cell of its region (no initial set is assumed):

* ``room1``: Room 1's free cells -> the move that A* takes toward NN1's goal, the Room 1 cell directly north of
  the doorway. At that goal cell the label is ``So`` (step into the doorway), never ``XX``.
* ``door``: the doorway cell -> ``So`` (step into Room 2).
* ``room2``: Room 2's free cells -> the A* move toward NN2's goal, the free cell nearest the middle of Room 2's box.
  At the goal the label is ``XX`` (stay).

Interface (the one BehaVerify's ``NEURAL classification`` variables use, as in the NeuS 2025 grid world):

* input: one float32 tensor ``inputs`` of shape ``[1, 2]`` = the drone's WHOLE-LAB cell ``(x, y)`` (x east, y north),
  raw integers, no scaling (the training scale is folded into the first layer);
* output: one float32 tensor ``scores`` of shape ``[1, 5]``, read by argmax in the order ``We, Ea, No, So, XX``;
* layers: ``Gemm`` and ``Relu`` only, the two operations BehaVerify's nuXmv translation supports.

A* runs inside the network's own region (4-connected, unit cost, Manhattan heuristic), so a label never leaves the
region. Every label is checked to be an OPTIMAL first move (it lowers the breadth-first distance to the goal by one).

After export, ``check`` re-reads each ONNX file with onnxruntime and checks, on every cell of its region: the argmax
equals the A* label, the winning score beats the runner-up by ``--margin``, and the closed loop (the region's network
chosen by the cell the drone is in, one move per tick) goes from every Room 1 cell to NN2's goal without entering a
cell outside the three regions. This is an exhaustive check over a finite domain in python, not a model-checking proof.

Writes, next to this script: ``room1.onnx``, ``door.onnx``, ``room2.onnx``, ``astar_labels.json`` (every label and
both goals), ``training_report.json`` (the checks) and ``astar_policy.png``.

    ~/venvs/nsbt/bin/python networks/train_astar_networks.py      # about 10 s; exit 0 = every check passed
"""
from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "grid"))
from lab_grid import LAB_GRID_JSON, LabGrid, load_lab_grid  # noqa: E402

ACTIONS = ("We", "Ea", "No", "So", "XX")
MOVE = {"We": (-1, 0), "Ea": (1, 0), "No": (0, 1), "So": (0, -1), "XX": (0, 0)}
A_STAR_ORDER = ("No", "Ea", "So", "We")      # neighbour expansion order: fixes A*'s choice among equal paths
NET_FILES = {"room1": "room1.onnx", "door": "door.onnx", "room2": "room2.onnx"}


# ---------------------------------------------------------------- the labels

def bfs_dist(cells: set, goal) -> dict:
    dist, q = {goal: 0}, deque([goal])
    while q:
        c = q.popleft()
        for m in A_STAR_ORDER:
            n = (c[0] + MOVE[m][0], c[1] + MOVE[m][1])
            if n in cells and n not in dist:
                dist[n] = dist[c] + 1
                q.append(n)
    return dist


def a_star_first_move(cells: set, start, goal):
    """First move of the A* path from start to goal inside ``cells`` (None if unreachable)."""
    h = lambda c: abs(c[0] - goal[0]) + abs(c[1] - goal[1])
    tie = 0
    openh = [(h(start), 0, tie, start)]
    came, g = {start: None}, {start: 0}
    while openh:
        _, gc, _, c = heapq.heappop(openh)
        if c == goal:
            while came[c] is not None and came[c][0] != start:
                c = came[c][0]
            return came[c][1] if came[c] else None
        if gc > g[c]:
            continue
        for m in A_STAR_ORDER:
            n = (c[0] + MOVE[m][0], c[1] + MOVE[m][1])
            if n in cells and gc + 1 < g.get(n, 1 << 30):
                g[n] = gc + 1
                came[n] = (c, m)
                tie += 1
                heapq.heappush(openh, (gc + 1 + h(n), gc + 1, tie, n))
    return None


def label_region(cells: set, goal, goal_action: str) -> dict:
    dist = bfs_dist(cells, goal)
    unreachable = sorted(c for c in cells if c not in dist)
    if unreachable:
        raise SystemExit(f"cells that cannot reach the goal {goal}: {unreachable}")
    labels = {}
    for c in sorted(cells):
        if c == goal:
            labels[c] = goal_action
            continue
        m = a_star_first_move(cells, c, goal)
        n = (c[0] + MOVE[m][0], c[1] + MOVE[m][1])
        assert dist[n] == dist[c] - 1, f"A* move {m} at {c} is not optimal"
        labels[c] = m
    return labels, dist


# ---------------------------------------------------------------- the network

def train(cells_labels: dict, hidden: list[int], margin: float, seed: int, max_epochs: int, scale):
    import torch
    torch.manual_seed(seed)
    X = torch.tensor([c for c in cells_labels], dtype=torch.float32)
    Y = torch.tensor([ACTIONS.index(a) for a in cells_labels.values()])
    mean, std = torch.tensor(scale[0]), torch.tensor(scale[1])
    Xn = (X - mean) / std
    sizes = [2, *hidden, len(ACTIONS)]
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(torch.nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(torch.nn.ReLU())
    net = torch.nn.Sequential(*layers)
    opt = torch.optim.Adam(net.parameters(), lr=1e-2)
    for epoch in range(1, max_epochs + 1):
        opt.zero_grad()
        out = net(Xn)
        # cross-entropy, plus a hinge that pushes every true score at least `margin` above the runner-up
        true = out.gather(1, Y[:, None]).squeeze(1)
        other = out.masked_fill(torch.nn.functional.one_hot(Y, len(ACTIONS)).bool(), -1e9).max(1).values
        loss = torch.nn.functional.cross_entropy(out, Y) + torch.relu(margin * 1.5 - (true - other)).mean()
        loss.backward()
        opt.step()
        if epoch % 100 == 0 or epoch == max_epochs:
            with torch.no_grad():
                out = net(Xn)
                true = out.gather(1, Y[:, None]).squeeze(1)
                other = out.masked_fill(torch.nn.functional.one_hot(Y, len(ACTIONS)).bool(), -1e9).max(1).values
                if bool(((true - other) >= margin).all()):
                    break
    # fold the input scaling into the first layer: W (x - m) / s + b = (W / s) x + (b - W m / s)
    weights = []
    with torch.no_grad():
        linears = [l for l in net if isinstance(l, torch.nn.Linear)]
        for i, l in enumerate(linears):
            W, b = l.weight.double().clone(), l.bias.double().clone()
            if i == 0:
                b = b - (W / std.double()) @ mean.double()
                W = W / std.double()
            weights.append((W.numpy().astype(np.float32), b.numpy().astype(np.float32)))
    return weights, epoch


def to_onnx(weights, name: str, path: Path, meta: dict):
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    nodes, inits, prev = [], [], "inputs"
    for i, (W, b) in enumerate(weights):
        inits += [numpy_helper.from_array(W, f"W{i}"), numpy_helper.from_array(b, f"B{i}")]
        out = "scores" if i == len(weights) - 1 else f"gemm{i}"
        nodes.append(helper.make_node("Gemm", [prev, f"W{i}", f"B{i}"], [out], transB=1, name=f"Gemm{i}"))
        if i < len(weights) - 1:
            nodes.append(helper.make_node("Relu", [out], [f"relu{i}"], name=f"Relu{i}"))
            prev = f"relu{i}"
    graph = helper.make_graph(nodes, name, [helper.make_tensor_value_info("inputs", TensorProto.FLOAT, [1, 2])],
                              [helper.make_tensor_value_info("scores", TensorProto.FLOAT, [1, len(ACTIONS)])], inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)], producer_name="vu-autonomy-compositional")
    model.ir_version = 8
    for k, v in meta.items():
        model.metadata_props.append(onnx.StringStringEntryProto(key=k, value=str(v)))
    onnx.checker.check_model(model)
    onnx.save(model, path)


# ---------------------------------------------------------------- the checks

def check(nets: dict, regions: dict, labels: dict, goals: dict, margin: float):
    import onnx
    import onnxruntime as ort
    report = {}
    sessions = {}
    for name, path in nets.items():
        ops = sorted({n.op_type for n in onnx.load(path).graph.node})
        assert set(ops) <= {"Gemm", "Relu"}, ops
        s = ort.InferenceSession(str(path))
        sessions[name] = s
        cells = sorted(regions[name])
        scores = np.concatenate([s.run(None, {"inputs": np.array([c], np.float32)})[0] for c in cells])
        pred = [ACTIONS[i] for i in scores.argmax(1)]
        wrong = [(c, p, labels[name][c]) for c, p in zip(cells, pred) if p != labels[name][c]]
        srt = np.sort(scores, 1)
        gaps = srt[:, -1] - srt[:, -2]
        # the call BehaVerify's generated python makes: a plain list of python ints
        py_call = s.run(None, {"inputs": [list(map(float, cells[0]))]})[0][0]
        report[name] = {"cells": len(cells), "wrong": [list(map(str, w)) for w in wrong],
                        "accuracy": 1 - len(wrong) / len(cells), "min_margin": float(gaps.min()),
                        "below_margin": int((gaps < margin).sum()), "ops": ops,
                        "python_list_call_ok": bool(np.argmax(py_call) == ACTIONS.index(labels[name][cells[0]]))}
    # closed loop: the network is chosen by the region the drone is in
    region_of = {c: n for n in ("room1", "door", "room2") for c in regions[n]}
    act = lambda c: ACTIONS[int(sessions[region_of[c]].run(None, {"inputs": np.array([c], np.float32)})[0].argmax())]
    final = goals["room2"]
    limit = sum(len(v) for v in regions.values()) + 1
    runs, fails = [], []
    for start in sorted(regions["room1"]) + sorted(regions["door"]) + sorted(regions["room2"]):
        c, steps, path = start, 0, [start]
        while True:
            a = act(c)
            if a == "XX":
                break
            c = (c[0] + MOVE[a][0], c[1] + MOVE[a][1])
            path.append(c)
            steps += 1
            if c not in region_of or steps > limit:
                break
        ok = c == final and act(c) == "XX"
        runs.append(steps)
        if not ok:
            fails.append({"start": start, "end": c, "steps": steps, "left_regions": c not in region_of})
    report["closed_loop"] = {"starts": len(runs), "reached_goal_and_stayed": len(runs) - len(fails),
                             "failures": fails, "max_steps": max(runs),
                             "from_room1_max_steps": max(runs[:len(regions["room1"])])}
    return report


# ---------------------------------------------------------------- figures

def plot(lab: LabGrid, labels: dict, goals: dict, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    INK, INK2, SURF, OBST, UNREACH = "#0b0b0b", "#52514e", "#fcfcfb", "#52514e", "#d6d5d0"
    R1, R2, DOOR, GOAL = "#86b6ef", "#f2a585", "#eda100", "#0ca30c"
    H, W = lab.H, lab.W
    code = np.zeros((H, W), int)                      # 0 obstacle, 1 unreachable free, 2 Room 1, 3 Room 2, 4 doorway
    for y in range(H):
        for x in range(W):
            code[y, x] = {0: 0, 1: 2, 2: 4, 3: 3, 4: 1}[lab.code(x, y)]
    fig, ax = plt.subplots(figsize=(10, 9.4))
    ax.imshow(code, cmap=ListedColormap([OBST, UNREACH, R1, R2, DOOR]), vmin=0, vmax=4, origin="lower",
              interpolation="nearest")
    ax.set_xticks(np.arange(-0.5, W, 1), minor=True); ax.set_yticks(np.arange(-0.5, H, 1), minor=True)
    ax.grid(which="minor", color=SURF, lw=0.8); ax.tick_params(which="minor", length=0)
    ax.set_xticks(range(0, W, 5)); ax.set_yticks(range(0, H, 5))
    ax.set_xlabel("x (east)", color=INK2); ax.set_ylabel("y (north)", color=INK2)
    for name in ("room1", "door", "room2"):
        for (x, y), a in labels[name].items():
            if a == "XX":
                ax.plot(x, y, "o", ms=7, color=INK)
            else:
                dx, dy = MOVE[a]
                ax.annotate("", xy=(x + 0.36 * dx, y + 0.36 * dy), xytext=(x - 0.36 * dx, y - 0.36 * dy),
                            arrowprops={"arrowstyle": "-|>", "color": INK, "lw": 1.3, "mutation_scale": 11})
    for name, lab in (("room1", "NN1 goal"), ("room2", "NN2 goal")):
        gx, gy = goals[name]
        ax.add_patch(plt.Rectangle((gx - 0.5, gy - 0.5), 1, 1, fill=False, lw=2.6, edgecolor=GOAL))
        ax.text(gx + 0.6, gy + 0.25, lab, color=INK, fontsize=9, fontweight="bold",
                bbox={"facecolor": SURF, "edgecolor": "none", "pad": 1.2})
    ax.set_title("A* labels the three networks reproduce: NN1 (Room 1), doorway, NN2 (Room 2)", loc="left",
                 color=INK, fontsize=11)
    ax.legend(handles=[Patch(facecolor=R1, label="Room 1: NN1"), Patch(facecolor=DOOR, label="doorway: NN_door"),
                       Patch(facecolor=R2, label="Room 2: NN2"), Patch(facecolor=OBST, label="obstacle"),
                       Patch(facecolor="none", edgecolor=GOAL, lw=2, label="goal cell"),
                       plt.Line2D([], [], marker="o", ls="", color=INK, label="XX (stay)")],
              loc="lower right", fontsize=9, framealpha=0.95)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", type=Path, default=LAB_GRID_JSON)
    ap.add_argument("--out", type=Path, default=HERE)
    ap.add_argument("--hidden", default="32,32", help="hidden layer widths for the room networks")
    ap.add_argument("--door-hidden", default="4", help="hidden layer widths for the doorway network")
    ap.add_argument("--margin", type=float, default=1.0, help="required score gap between the label and the runner-up")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-epochs", type=int, default=50000)
    a = ap.parse_args()

    lab = load_lab_grid(a.grid)
    regions, door = lab.regions, lab.door
    goal1 = (door[0], door[1] + 1)
    if goal1 not in regions["room1"]:
        raise SystemExit(f"the cell north of the doorway {goal1} is not in Room 1")
    goal2 = lab.room_middle("room2")
    goals = {"room1": goal1, "door": door, "room2": goal2}
    labels, dists = {}, {}
    labels["room1"], dists["room1"] = label_region(regions["room1"], goal1, "So")
    labels["door"] = {door: "So"}
    labels["room2"], dists["room2"] = label_region(regions["room2"], goal2, "XX")
    entry2 = (door[0], door[1] - 1)
    assert entry2 in regions["room2"], "the cell south of the doorway is not in Room 2"

    H, W = lab.H, lab.W
    scale = ([(W - 1) / 2, (H - 1) / 2], [(W - 1) / 2, (H - 1) / 2])
    files = NET_FILES
    a.out.mkdir(parents=True, exist_ok=True)
    trained = {}
    for name, fname in files.items():
        hidden = [int(h) for h in (a.door_hidden if name == "door" else a.hidden).split(",") if h]
        weights, epochs = train(labels[name], hidden, a.margin, a.seed, a.max_epochs, scale)
        meta = {"region": name, "goal_xy": goals[name], "inputs": "(x, y) whole-lab cell, x east, y north",
                "outputs": "argmax over We, Ea, No, So, XX", "grid_sha256": hashlib.sha256(a.grid.read_bytes()).hexdigest()}
        to_onnx(weights, files[name].removesuffix(".onnx"), a.out / fname, meta)
        trained[name] = {"file": fname, "layers": [2, *hidden, len(ACTIONS)], "epochs": epochs,
                         "params": int(sum(W.size + b.size for W, b in weights))}
        print(f"{fname}: layers {trained[name]['layers']}, {epochs} epochs")

    rep = check({n: a.out / f for n, f in files.items()}, regions, labels, goals, a.margin)

    # negative controls: two broken copies of NN1 that the same check must reject
    import tempfile
    import onnx
    from onnx import numpy_helper
    controls = {}
    with tempfile.TemporaryDirectory() as tmp:
        for cname, (action, bump) in {"nn1_biased_west": ("We", 3.0), "nn1_hovers": ("XX", 50.0)}.items():
            m = onnx.load(a.out / files["room1"])
            last = [i for i in m.graph.initializer if i.name.startswith("B")][-1]
            b = numpy_helper.to_array(last).copy()
            b[ACTIONS.index(action)] += bump
            last.CopyFrom(numpy_helper.from_array(b, last.name))
            onnx.save(m, Path(tmp) / "nn1.onnx")
            bad = check({**{n: a.out / f for n, f in files.items()}, "room1": Path(tmp) / "nn1.onnx"},
                        regions, labels, goals, a.margin)
            fails = bad["closed_loop"]["failures"]
            controls[cname] = {"change": f"+{bump} on the {action} score of NN1's last layer",
                               "nn1_wrong_cells": len(bad["room1"]["wrong"]),
                               "closed_loop_failures": len(fails), "rejected": len(fails) > 0,
                               "first_failure": fails[0] if fails else None}
    rep["negative_controls"] = controls
    table = {n: {f"{x},{y}": lab for (x, y), lab in sorted(labels[n].items())} for n in files}
    (a.out / "astar_labels.json").write_text(json.dumps({
        "actions": list(ACTIONS), "moves": {k: list(v) for k, v in MOVE.items()},
        "goals_xy": {n: list(g) for n, g in goals.items()}, "room2_entry_xy": list(entry2),
        "a_star": {"connectivity": 4, "cost": 1, "heuristic": "manhattan", "expansion_order": list(A_STAR_ORDER),
                   "inside": "the network's own region only"},
        "labels_by_xy": table}, indent=1) + "\n")
    (a.out / "training_report.json").write_text(json.dumps({
        "grid": str(a.grid.resolve().relative_to(HERE.parent)) if a.grid.resolve().is_relative_to(HERE.parent) else str(a.grid),
        "grid_sha256": hashlib.sha256(a.grid.read_bytes()).hexdigest(),
        "margin_required": a.margin, "seed": a.seed, "networks": trained,
        "sha256": {f: hashlib.sha256((a.out / f).read_bytes()).hexdigest() for f in files.values()},
        "check": rep}, indent=1, default=str) + "\n")
    plot(lab, labels, goals, a.out / "astar_policy.png")

    for n in files:
        r = rep[n]
        print(f"  {n}: {r['cells']} cells, accuracy {r['accuracy']:.3f}, min margin {r['min_margin']:.2f}, "
              f"ops {r['ops']}, python-list call ok {r['python_list_call_ok']}")
    cl = rep["closed_loop"]
    print(f"  closed loop: {cl['reached_goal_and_stayed']}/{cl['starts']} starts reach NN2's goal; "
          f"longest from Room 1 {cl['from_room1_max_steps']} moves; failures {cl['failures'][:3]}")
    for cname, c in rep["negative_controls"].items():
        print(f"  negative control {cname}: {c['nn1_wrong_cells']} wrong cells, "
              f"{c['closed_loop_failures']} closed-loop failures -> rejected {c['rejected']}; e.g. {c['first_failure']}")
    ok = (all(rep[n]["accuracy"] == 1 and rep[n]["below_margin"] == 0 for n in files) and not cl["failures"]
          and all(c["rejected"] for c in rep["negative_controls"].values()))
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
