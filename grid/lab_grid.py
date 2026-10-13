"""Read ``grid/lab_grid.json`` (written by ``discretize_lab.py``) into the cell sets every later step uses.

Standard library only, so the training (torch), tree (BehaVerify) and grid environments can all import it:

    sys.path.insert(0, str(<repo>/"grid")); from lab_grid import load_lab_grid

Cells are ``(x, y)``: x east, y north, ``(0, 0)`` the south-west cell of the whole-lab grid.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

LAB_GRID_JSON = Path(__file__).resolve().parent / "lab_grid.json"

OBSTACLE, ROOM1, DOOR, ROOM2, UNREACHABLE = 0, 1, 2, 3, 4    # the codes of ``cellmap``
REGIONS = ("room1", "door", "room2")


@dataclass(frozen=True)
class LabGrid:
    W: int                                  # columns (x)
    H: int                                  # rows (y)
    door: tuple[int, int]
    regions: dict[str, frozenset]           # "room1" / "door" / "room2" -> its free cells
    cellmap: tuple[int, ...]                # cellmap[y * W + x]: OBSTACLE, ROOM1, DOOR, ROOM2 or UNREACHABLE
    raw: dict                               # the whole lab_grid.json

    def code(self, x: int, y: int) -> int:
        return self.cellmap[y * self.W + x] if 0 <= x < self.W and 0 <= y < self.H else OBSTACLE

    def region_of(self, cell) -> str | None:
        return {ROOM1: "room1", DOOR: "door", ROOM2: "room2"}.get(self.code(*cell))

    def room_middle(self, room: str) -> tuple[int, int]:
        """The free cell of ``room`` nearest the middle of its bounding box (ties: south, then west)."""
        r = self.raw["rooms"][room]
        ox, oy = r["origin_in_global_xy"]
        cx, cy = ox + (r["N_cols"] - 1) / 2, oy + (r["M_rows"] - 1) / 2
        return min(self.regions[room], key=lambda c: ((c[0] - cx) ** 2 + (c[1] - cy) ** 2, c[1], c[0]))


def load_lab_grid(path: Path = LAB_GRID_JSON) -> LabGrid:
    d = json.loads(Path(path).read_text())
    rows = d["global"]["rows_north_up"]
    H, W = len(rows), len(rows[0])
    regions = {}
    for name in ("room1", "room2"):
        r = d["rooms"][name]
        ox, oy = r["origin_in_global_xy"]
        regions[name] = frozenset((ox + x, oy + y) for x, y in r["free_xy"])
    door = tuple(d["door"]["xy"])
    regions["door"] = frozenset({door})
    code_of = {c: code for name, code in (("room1", ROOM1), ("door", DOOR), ("room2", ROOM2)) for c in regions[name]}
    cellmap = tuple(code_of.get((x, y), UNREACHABLE) if rows[H - 1 - y][x] != "#" else OBSTACLE
                    for y in range(H) for x in range(W))
    return LabGrid(W, H, door, {n: regions[n] for n in REGIONS}, cellmap, d)
