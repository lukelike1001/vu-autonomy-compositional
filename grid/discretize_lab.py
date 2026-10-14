#!/usr/bin/env python3
"""Discretize the lab's two-room footprint grid into a coarse 2-D grid world.

A rough abstraction for discretization tests, heights ignored. Input: a safenav ``footprint4d.npz``
(``free`` / ``blocked`` at one altitude z*, 0.05 m cells, site frame E/N). Output: a coarse grid in
which every cell is FREE or OBSTACLE, split into the two rooms joined by a one-cell doorway.

Rules:

* **Over-estimate obstacles.** A coarse cell is FREE only if every 0.05 m cell inside it is ``free``.
  Any ``blocked`` or unknown (neither free nor blocked) fine cell makes the whole coarse cell an
  obstacle, as does any part of the cell outside the map.
* **The cell size** is ``--cell`` (default 0.65 m, chosen to keep the networks small). For it, and for every other
  size in ``--min-cell`` to ``--max-cell`` (for the comparison figure), every grid offset is tried and the one with
  the shortest doorway, then the most free area reachable, is kept. ``--cell auto`` instead picks the LARGEST size at
  which exactly one cell separates the rooms (0.35 m).
* **The doorway is one cell, ``D``.** Removing a doorway cell disconnects the rooms (4-connected). At 0.65 m the
  doorway is a corridor of 5 such cells; the one nearest ``DOOR_POINT`` (the doorway's narrowest point) is ``D``, and
  every other corridor cell goes to the room it stays connected to.
* **Rooms.** Room 1 is the component holding ``--room1-seed`` (default: the map origin E 0, N 0, floor tag 39), Room 2
  the component holding ``--room2-seed`` (default: the room-transit end in the ICRA figure), once the
  door cell is removed. Each room's grid is its bounding box; anything in the box that is not that
  room's free space is an obstacle.

Coordinates in the output are cells ``(x, y)``: x counts EAST and y counts NORTH from the grid's south-west
cell (0, 0), as in the site frame (E, N) and ROS REP 103 (ENU). The
text grids are printed north-up like a map, so their LAST line is y = 0. ``.`` free, ``#`` obstacle, ``D`` the
doorway (whole-lab grid only). Free cells that no room can reach are kept as ``.`` and counted.

Writes, next to this script: ``lab_grid.json`` (everything, with the source's repository, path, sha256 and commit),
``lab_grid.txt``, ``room1_grid.txt``, ``room2_grid.txt``, ``lab_grid.png`` and ``cell_size_comparison.png``.

    ~/venvs/vu-flight/bin/python grid/discretize_lab.py        # about a minute
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from collections import deque
from pathlib import Path

import numpy as np

ROOM1_SEED = (0.0, 0.0)     # (E, N) metres: the pad
ROOM2_SEED = (0.1, -5.6)    # (E, N) metres: the end of the room transit
HERE = Path(__file__).resolve().parent
FOOTPRINT = HERE.parent.parent / "vu-autonomy" / "scripts/flights/slices/v6.2-transit-z155-w030-v020/footprint4d.npz"
DOOR_POINT = (2.475, -2.70)  # (E, N) metres: the doorway's narrowest free row (E 2.05 to 2.90 at N -2.8 to -2.6)
SEED_SEARCH_M = 1.0         # a seed whose own cell is an obstacle moves to the nearest free cell within this


def coarsen(free: np.ndarray, k: int, oy: int, ox: int) -> np.ndarray:
    """Coarse FREE mask, row 0 = south (the fine grid's orientation). Out-of-map counts as not free."""
    H, W = free.shape
    nr, nc = -(-(H + oy) // k), -(-(W + ox) // k)
    pad = np.zeros((nr * k, nc * k), bool)
    pad[oy:oy + H, ox:ox + W] = free
    return pad.reshape(nr, k, nc, k).all(axis=(1, 3))


def component(G: np.ndarray, seed: tuple[int, int]) -> set[tuple[int, int]]:
    if not (0 <= seed[0] < G.shape[0] and 0 <= seed[1] < G.shape[1]) or not G[seed]:
        return set()
    seen, q = {seed}, deque([seed])
    while q:
        y, x = q.popleft()
        for p in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= p[0] < G.shape[0] and 0 <= p[1] < G.shape[1] and G[p] and p not in seen:
                seen.add(p)
                q.append(p)
    return seen


class Grid:
    """One coarse grid: cell size k fine cells, offset (oy, ox) fine cells."""

    def __init__(self, fp, k: int, oy: int, ox: int, seeds):
        self.k, self.oy, self.ox = k, oy, ox
        self.res, self.E0, self.N0 = fp["res"], fp["E0"], fp["N0"]
        self.G = coarsen(fp["free"], k, oy, ox)
        self.seeds = [self._seed(e, n) for e, n in seeds]

    def cell_of(self, e: float, n: float) -> tuple[int, int]:
        return (int((n - self.N0) / self.res) + self.oy) // self.k, (int((e - self.E0) / self.res) + self.ox) // self.k

    def _seed(self, e, n):
        c = self.cell_of(e, n)
        reach = int(np.ceil(SEED_SEARCH_M / (self.k * self.res)))
        cands = [(abs(dy) + abs(dx), (c[0] + dy, c[1] + dx)) for dy in range(-reach, reach + 1)
                 for dx in range(-reach, reach + 1)]
        for _, p in sorted(cands):
            if 0 <= p[0] < self.G.shape[0] and 0 <= p[1] < self.G.shape[1] and self.G[p]:
                return p
        return None

    def door_cells(self) -> list[tuple[int, int]]:
        """Cells whose removal disconnects the two seeds (empty if they are not connected)."""
        a, b = self.seeds
        if a is None or b is None:
            return []
        A = component(self.G, a)
        if b not in A:
            return []
        out = []
        for c in A:
            if c in (a, b):
                continue
            G2 = self.G.copy()
            G2[c] = False
            if b not in component(G2, a):
                out.append(c)
        return out

    def cell_area(self) -> float:
        return (self.k * self.res) ** 2


def search(fp, seeds, sizes):
    """For each cell size: the best offset (one-cell door first, then most reachable area)."""
    table = []
    for k in sizes:
        best = None
        for oy in range(k):
            for ox in range(k):
                g = Grid(fp, k, oy, ox, seeds)
                D = g.door_cells()
                if not D:
                    continue
                area = len(component(g.G, g.seeds[0])) * g.cell_area()
                key = (len(D) == 1, -len(D), area)
                if best is None or key > best[0]:
                    best = (key, g, D, area)
        if best:
            table.append({"k": k, "cell_m": round(k * fp["res"], 4), "grid": best[1], "door": best[2],
                          "door_chain": len(best[2]), "reachable_m2": round(best[3], 2)})
    return table


def to_rows(mask_south_up: np.ndarray) -> np.ndarray:
    """Flip so row 0 is north."""
    return mask_south_up[::-1]


def build(fp, g: Grid, door):
    """Global grid and per-room grids, all north-up."""
    nr = g.G.shape[0]
    flip = lambda c: (nr - 1 - c[0], c[1])
    G = g.G.copy()
    (d,) = door
    G[d] = False
    rooms = {}
    for name, seed in zip(("room1", "room2"), g.seeds):
        rooms[name] = {flip(c) for c in component(G, seed)}
    Gn = to_rows(g.G)
    chars = np.where(Gn, ".", "#").astype("<U1")
    dn = flip(d)
    chars[dn] = "D"
    reached = rooms["room1"] | rooms["room2"] | {dn}
    unreachable = int(sum(1 for y, x in zip(*np.nonzero(Gn)) if (y, x) not in reached))
    cell = g.k * g.res
    # site-frame E/N of the coarse grid's north-west corner (row 0, col 0)
    west_E = g.E0 - g.ox * g.res
    north_N = g.N0 - g.oy * g.res + nr * cell
    out_rooms = {}
    for name, cells in rooms.items():
        ys, xs = zip(*cells)
        r0, r1, c0, c1 = min(ys), max(ys), min(xs), max(xs)
        sub = np.full((r1 - r0 + 1, c1 - c0 + 1), "#", dtype="<U1")
        for y, x in cells:
            sub[y - r0, x - c0] = "."
        xy = lambda rc: [int(rc[1] - c0), int(r1 - rc[0])]          # room-local (x east, y north)
        nbr = [xy(p) for p in ((dn[0] + 1, dn[1]), (dn[0] - 1, dn[1]), (dn[0], dn[1] + 1), (dn[0], dn[1] - 1))
               if p in cells]
        out_rooms[name] = {
            "M_rows": int(sub.shape[0]), "N_cols": int(sub.shape[1]),
            "free_cells": len(cells), "obstacle_cells": int(sub.size - len(cells)),
            "origin_in_global_xy": [int(c0), int(nr - 1 - r1)],
            "sw_corner_EN_m": [round(west_E + c0 * cell, 4), round(north_N - (r1 + 1) * cell, 4)],
            "door_neighbour_xy": nbr,
            "free_xy": sorted(xy(c) for c in cells),
            "rows_north_up": ["".join(r) for r in sub],
            "_rc": (int(r0), int(c0)),                                  # internal, not written
        }
    for name, cells in rooms.items():
        out_rooms[name]["_cells"] = cells
    overlap = _bbox_overlap(out_rooms)
    return chars, dn, out_rooms, unreachable, (west_E, north_N), overlap


def _bbox_overlap(rooms):
    (a, b) = rooms.values()
    ar, ac = a["_rc"]
    br, bc = b["_rc"]
    rows = max(ar, br) <= min(ar + a["M_rows"], br + b["M_rows"]) - 1
    cols = max(ac, bc) <= min(ac + a["N_cols"], bc + b["N_cols"]) - 1
    return bool(rows and cols)


def git_commit(path: Path):
    try:
        return subprocess.run(["git", "-C", str(path.parent), "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def git_relpath(path: Path) -> str:
    try:
        top = subprocess.run(["git", "-C", str(path.parent), "rev-parse", "--show-toplevel"], capture_output=True,
                             text=True, check=True).stdout.strip()
        return str(path.relative_to(top))
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        return path.name


def git_remote(path: Path):
    try:
        url = subprocess.run(["git", "-C", str(path.parent), "remote", "get-url", "origin"], capture_output=True,
                             text=True, check=True).stdout.strip()
        return url.split(":")[-1].removesuffix(".git").split("github.com/")[-1]
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def plot(fp, g: Grid, chars, dn, rooms, table, chosen_k, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch, Rectangle

    INK, INK2, SURF = "#0b0b0b", "#52514e", "#fcfcfb"
    OBST, UNREACH, UNKNOWN = "#52514e", "#d6d5d0", "#b9b8b2"
    R1, R2, DOOR = "#2a78d6", "#eb6834", "#eda100"
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF})

    def grid_codes(chars, rooms_cells):
        code = np.where(chars == "#", 0, 1)            # 0 obstacle, 1 unreachable free
        for i, cells in enumerate(rooms_cells, start=2):  # 2 room1, 3 room2
            for y, x in cells:
                code[y, x] = i
        code[chars == "D"] = 4
        return code

    cmap = ListedColormap([OBST, UNREACH, R1, R2, DOOR])

    def room_cells(rooms):
        out = []
        for r in rooms.values():
            r0, c0 = r["_rc"]
            out.append([(r0 + y, c0 + x) for y, row in enumerate(r["rows_north_up"]) for x, ch in enumerate(row)
                        if ch == "." and (r0 + y, c0 + x) in r["_cells"]])
        return out

    cell = g.k * g.res
    west_E, north_N = g.E0 - g.ox * g.res, g.N0 - g.oy * g.res + g.G.shape[0] * cell
    nr, nc = chars.shape

    # ---- figure 1: the survey grid with the coarse grid on it, and the grid world itself
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 7.6), gridspec_kw={"wspace": 0.18})
    fine = np.full(fp["free"].shape, 2)           # 2 unknown
    fine[fp["free"]] = 0
    fine[fp["blocked"]] = 1
    fext = [fp["E0"], fp["E0"] + fine.shape[1] * fp["res"], fp["N0"], fp["N0"] + fine.shape[0] * fp["res"]]
    a1.imshow(fine, origin="lower", extent=fext, cmap=ListedColormap([SURF, OBST, UNKNOWN]),
              interpolation="nearest", vmin=0, vmax=2)
    code = grid_codes(chars, room_cells(rooms))
    for y in range(nr):
        for x in range(nc):
            if code[y, x] in (2, 3, 4):
                colr = {2: R1, 3: R2, 4: DOOR}[code[y, x]]
                a1.add_patch(Rectangle((west_E + x * cell, north_N - (y + 1) * cell), cell, cell,
                                       facecolor=colr, alpha=0.35 if code[y, x] != 4 else 0.8,
                                       edgecolor="none"))
    for x in range(nc + 1):
        a1.axvline(west_E + x * cell, color=INK2, lw=0.3, alpha=0.5)
    for y in range(nr + 1):
        a1.axhline(north_N - y * cell, color=INK2, lw=0.3, alpha=0.5)
    a1.set_xlim(fext[0], fext[1]); a1.set_ylim(fext[2], fext[3]); a1.set_aspect("equal")
    a1.set_xlabel("E [m]"); a1.set_ylabel("N [m]")
    a1.set_title(f"Survey grid at z* = {fp['z_star']:.2f} m (0.05 m cells) under the {cell:.2f} m grid",
                 color=INK, fontsize=11, loc="left")
    a1.legend(handles=[Patch(facecolor=SURF, edgecolor=INK2, label="free (fine)"),
                       Patch(facecolor=OBST, label="blocked (fine)"),
                       Patch(facecolor=UNKNOWN, label="unknown (fine)"),
                       Patch(facecolor=R1, alpha=0.35, label="Room 1 cell"),
                       Patch(facecolor=R2, alpha=0.35, label="Room 2 cell"),
                       Patch(facecolor=DOOR, alpha=0.8, label="doorway cell")],
              loc="lower right", fontsize=8.5, framealpha=0.95)

    a2.imshow(code[::-1], cmap=cmap, vmin=0, vmax=4, interpolation="nearest", origin="lower")
    a2.set_xticks(np.arange(-0.5, nc, 1), minor=True); a2.set_yticks(np.arange(-0.5, nr, 1), minor=True)
    a2.grid(which="minor", color=SURF, lw=1.0)
    a2.tick_params(which="minor", length=0)
    a2.set_xticks(range(0, nc, 5)); a2.set_yticks(range(0, nr, 5))
    a2.set_xlabel("x (east)"); a2.set_ylabel("y (north)")
    a2.text(dn[1], nr - 1 - dn[0], "D", ha="center", va="center", fontsize=8, fontweight="bold", color=INK)
    for (name, r), colr in zip(rooms.items(), (R1, R2)):
        r0, c0 = r["_rc"]
        a2.add_patch(Rectangle((c0 - 0.5, nr - r0 - r["M_rows"] - 0.5), r["N_cols"], r["M_rows"], fill=False, lw=2.0,
                               edgecolor=SURF, ls="--"))
        label = f"{'Room 1' if name == 'room1' else 'Room 2'}: {r['M_rows']} × {r['N_cols']}"
        a2.text(c0 - 0.3, nr - 1 - r0 + 0.9, label, fontsize=10, color=INK, fontweight="bold", va="bottom",
                bbox={"facecolor": SURF, "edgecolor": "none", "pad": 1.5})
    a2.set_title(f"Grid world: {cell:.2f} m cells, {nr} × {nc}, doorway = 1 cell",
                 color=INK, fontsize=11, loc="left")
    a2.legend(handles=[Patch(facecolor=OBST, label="obstacle"), Patch(facecolor=R1, label="Room 1 free"),
                       Patch(facecolor=R2, label="Room 2 free"), Patch(facecolor=DOOR, label="doorway (D)"),
                       Patch(facecolor=UNREACH, label="free, unreachable"),
                       Patch(facecolor=OBST, edgecolor=SURF, ls="--", label="room M × N box")],
              loc="lower right", fontsize=8.5, framealpha=0.95)
    fig.savefig(out / "lab_grid.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    # ---- figure 2: the cell-size trade-off, small multiples
    show = [t for t in table if t["k"] in (7, 10, 13) or t["k"] == chosen_k]       # 0.35, 0.50, 0.65 m
    fig, axes = plt.subplots(1, len(show), figsize=(4.2 * len(show), 4.9))
    for ax, t in zip(np.atleast_1d(axes), show):
        gg = t["grid"]
        if len(t["door"]) == 1:
            ch, _, rr, *_ = build(fp, gg, t["door"])
            cd = grid_codes(ch, room_cells(rr))
        else:   # a corridor of door cells: colour them all as doorway
            nrr = gg.G.shape[0]
            ch = np.where(to_rows(gg.G), ".", "#").astype("<U1")
            cd = np.where(ch == "#", 0, 1)
            G2 = gg.G.copy()
            for c in t["door"]:
                G2[c] = False
            for i, s in enumerate(gg.seeds, start=2):
                for (y, x) in component(G2, s):
                    cd[nrr - 1 - y, x] = i
            for (y, x) in t["door"]:
                cd[nrr - 1 - y, x] = 4
        ax.imshow(cd[::-1], cmap=cmap, vmin=0, vmax=4, interpolation="nearest", origin="lower")
        ax.set_xticks([]); ax.set_yticks([])
        tag = " (chosen)" if t["k"] == chosen_k else ""
        ax.set_title(f"{t['cell_m']:.2f} m cells{tag}\ndoorway {t['door_chain']} cell(s) long · "
                     f"{t['reachable_m2']:.0f} m² reachable", fontsize=10, color=INK, loc="left")
    fig.suptitle("Larger cells: the one-cell doorway becomes a corridor and the rooms shrink",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.savefig(out / "cell_size_comparison.png", dpi=140, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--footprint", type=Path, default=FOOTPRINT,
                    help="safenav footprint4d.npz (default: the doorway-transit slice in a sibling ttj/vu-autonomy clone)")
    ap.add_argument("--cell", default="0.65", help="cell size in metres (a multiple of the map res), or 'auto'")
    ap.add_argument("--min-cell", type=float, default=0.30)
    ap.add_argument("--max-cell", type=float, default=0.75, help="0.80 m and up disconnect the rooms")
    ap.add_argument("--out", type=Path, default=HERE)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    d = np.load(a.footprint)
    fp = {"free": d["free"].astype(bool), "blocked": d["blocked"].astype(bool), "res": float(d["res"]),
          "E0": float(d["origin"][0]), "N0": float(d["origin"][1]), "z_star": float(d["z_star"])}
    seeds = (ROOM1_SEED, ROOM2_SEED)
    kmin, kmax = round(a.min_cell / fp["res"]), round(a.max_cell / fp["res"])
    table = search(fp, seeds, range(kmin, kmax + 1))
    if not table:
        raise SystemExit("no cell size in range keeps the two rooms connected")
    if a.cell == "auto":
        ones = [t for t in table if t["door_chain"] == 1]
        if not ones:
            raise SystemExit("no cell size in range gives a one-cell doorway")
        chosen = max(ones, key=lambda t: t["k"])
    else:
        k = round(float(a.cell) / fp["res"])
        chosen = next((t for t in table if t["k"] == k), None)
        if chosen is None:
            raise SystemExit(f"cell {a.cell} m does not keep the rooms connected; sizes that do: "
                             f"{[t['cell_m'] for t in table]}")
    g = chosen["grid"]
    target = g.cell_of(*DOOR_POINT)
    door = min(chosen["door"], key=lambda c: (c[0] - target[0]) ** 2 + (c[1] - target[1]) ** 2)
    chars, dn, rooms, unreachable, (west_E, north_N), overlap = build(fp, g, [door])
    nr = g.G.shape[0]
    corridor = {}
    for name, r in rooms.items():
        corridor[name] = sorted([c[1], c[0]] for c in chosen["door"] if (nr - 1 - c[0], c[1]) in r["_cells"])

    src = a.footprint.resolve()
    result = {
        "schema": "lab-grid-world/1",
        "claim": "A rough 2-D abstraction of ONE survey-derived grid at one altitude, heights ignored. "
                 "Not a statement about the room: furniture moves and the survey has holes.",
        "source": {"repo": git_remote(src), "footprint": git_relpath(src), "sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
                   "git_commit": git_commit(src), "z_star_m": fp["z_star"], "fine_res_m": fp["res"],
                   "fine_origin_EN_m": [fp["E0"], fp["N0"]], "layer": "free (blocked and unknown = obstacle)"},
        "params": {"cell_m": chosen["cell_m"], "cell_fine": g.k, "offset_fine_rows_cols": [g.oy, g.ox],
                   "rule": "coarse cell FREE iff every fine cell in it is free",
                   "connectivity": 4, "room1_seed_EN_m": list(ROOM1_SEED), "room2_seed_EN_m": list(ROOM2_SEED),
                   "selection": ("largest cell size with exactly one door cell" if a.cell == "auto" else
                                 f"cell size {a.cell} m as requested") + "; offset: shortest doorway, then most reachable area"},
        "conventions": {"cell": "(x, y): x east, y north, (0, 0) = the south-west cell",
                        "moves": {"E": [1, 0], "N": [0, 1], "W": [-1, 0], "S": [0, -1]},
                        "text": "rows_north_up: first string is the NORTH row, last is y = 0",
                        "free": ".", "obstacle": "#", "door": "D",
                        "sw_corner_EN_m": [round(west_E, 4), round(north_N - chars.shape[0] * g.k * g.res, 4)]},
        "global": {"M_rows": int(chars.shape[0]), "N_cols": int(chars.shape[1]),
                   "rows_north_up": ["".join(r) for r in chars]},
        "door": {"xy": [int(dn[1]), int(nr - 1 - dn[0])],
                 "center_EN_m": [round(west_E + (dn[1] + 0.5) * g.k * g.res, 3),
                                 round(north_N - (dn[0] + 0.5) * g.k * g.res, 3)],
                 "corridor_cells": chosen["door_chain"],
                 "corridor_cells_given_to_rooms_global_xy": corridor},
        "rooms": {n: {k: v for k, v in r.items() if not k.startswith("_")} for n, r in rooms.items()},
        "unreachable_free_cells": unreachable,
        "room_boxes_overlap": overlap,
        "cell_size_search": [{k: v for k, v in t.items() if k not in ("grid", "door")} for t in table],
    }
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "lab_grid.json").write_text(json.dumps(result, indent=1) + "\n")
    for name in ("room1", "room2"):
        (a.out / f"{name}_grid.txt").write_text("\n".join(rooms[name]["rows_north_up"]) + "\n")
    (a.out / "lab_grid.txt").write_text("\n".join(result["global"]["rows_north_up"]) + "\n")
    if not a.no_plot:
        plot(fp, g, chars, dn, rooms, table, chosen["k"], a.out)

    print(f"doorway corridor {chosen['door_chain']} cell(s); given to rooms: {corridor}")
    print(f"cell {chosen['cell_m']} m, global {chars.shape[0]} x {chars.shape[1]}, door at xy {result['door']['xy']}")
    for name, r in rooms.items():
        print(f"  {name}: M x N = {r['M_rows']} x {r['N_cols']}  free {r['free_cells']}  obstacle {r['obstacle_cells']}")
    print(f"  unreachable free cells: {unreachable}; room boxes overlap: {overlap}")
    for t in table:
        print(f"  search {t['cell_m']:.2f} m: door {t['door_chain']} cell(s), {t['reachable_m2']} m2 reachable")


if __name__ == "__main__":
    main()
