#!/usr/bin/env python3
"""
Grid-based A* path planner for the medical-delivery drone.

Coordinates are metres in the local NED frame used by ArduPilot:
    north = +x (forward from home), east = +y (right of home).
Home / the hospital is (0, 0).

Pipeline:
    1. Build an occupancy grid from the map bounds (the flight boundary).
    2. Mark every obstacle cell, inflated by a safety margin.
    3. Run A* (8-connected, octile heuristic, no corner cutting).
    4. Simplify the cell path into a few straight-line waypoints
       using line-of-sight checks on the same grid.

Run directly to print the planned route and save a preview image:
    python3 scripts/planner.py
"""
from __future__ import annotations

import heapq
import json
import math
import os
import sys
from dataclasses import dataclass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(ROOT, "config", "mission.json")

SQRT2 = math.sqrt(2.0)


class PlanningError(Exception):
    """Raised when no collision-free path exists."""


@dataclass
class Box:
    """Axis-aligned rectangular obstacle (footprint in the N/E plane)."""
    name: str
    north: float
    east: float
    size_n: float
    size_e: float
    height: float = 30.0

    def contains(self, n: float, e: float, margin: float = 0.0) -> bool:
        return (abs(n - self.north) <= self.size_n / 2 + margin and
                abs(e - self.east) <= self.size_e / 2 + margin)

    def distance_to(self, n: float, e: float) -> float:
        """Distance from a point to the box footprint (0 if inside)."""
        dn = max(abs(n - self.north) - self.size_n / 2, 0.0)
        de = max(abs(e - self.east) - self.size_e / 2, 0.0)
        return math.hypot(dn, de)


def load_config(path: str = DEFAULT_CONFIG) -> dict:
    with open(path) as f:
        return json.load(f)


def obstacles_from_config(cfg: dict) -> list[Box]:
    return [Box(o["name"], o["north"], o["east"], o["size_n"], o["size_e"], o.get("height", 30.0))
            for o in cfg["obstacles"]]


class GridMap:
    """Occupancy grid. Cell (i, j) <-> point (n_min + i*res, e_min + j*res)."""

    def __init__(self, bounds: dict, resolution: float, margin: float):
        self.n_min = float(bounds["north_min"])
        self.n_max = float(bounds["north_max"])
        self.e_min = float(bounds["east_min"])
        self.e_max = float(bounds["east_max"])
        self.res = float(resolution)
        self.margin = float(margin)
        self.rows = int(round((self.n_max - self.n_min) / self.res)) + 1
        self.cols = int(round((self.e_max - self.e_min) / self.res)) + 1
        self.blocked = bytearray(self.rows * self.cols)

    # --- conversions -----------------------------------------------------
    def to_cell(self, n: float, e: float) -> tuple[int, int]:
        return (int(round((n - self.n_min) / self.res)),
                int(round((e - self.e_min) / self.res)))

    def to_point(self, i: int, j: int) -> tuple[float, float]:
        return (self.n_min + i * self.res, self.e_min + j * self.res)

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.rows and 0 <= j < self.cols

    def is_free_cell(self, i: int, j: int) -> bool:
        return self.in_bounds(i, j) and not self.blocked[i * self.cols + j]

    def is_free(self, n: float, e: float) -> bool:
        return self.is_free_cell(*self.to_cell(n, e))

    # --- building the map ------------------------------------------------
    def add_box(self, box: Box) -> None:
        """Mark all cells inside the box inflated by the safety margin."""
        half_n = box.size_n / 2 + self.margin
        half_e = box.size_e / 2 + self.margin
        i0, j0 = self.to_cell(box.north - half_n, box.east - half_e)
        i1, j1 = self.to_cell(box.north + half_n, box.east + half_e)
        for i in range(max(i0, 0), min(i1, self.rows - 1) + 1):
            row = i * self.cols
            for j in range(max(j0, 0), min(j1, self.cols - 1) + 1):
                self.blocked[row + j] = 1

    def line_free(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """True if the straight segment a->b only crosses free cells."""
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(dist / (self.res * 0.5))))
        for k in range(steps + 1):
            t = k / steps
            if not self.is_free(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
                return False
        return True


def build_grid(cfg: dict, extra_obstacles: list[Box] | None = None) -> GridMap:
    grid = GridMap(cfg["map_bounds"], cfg["grid_resolution"], cfg["safety_margin"])
    for box in obstacles_from_config(cfg) + list(extra_obstacles or []):
        grid.add_box(box)
    return grid


# --------------------------------------------------------------------------
# A*
# --------------------------------------------------------------------------
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def _nearest_free_cell(grid: GridMap, cell: tuple[int, int], max_radius: int = 30):
    """Breadth-first ring search for the closest free cell (used when the
    drone currently sits inside an inflated safety margin)."""
    ci, cj = cell
    for r in range(1, max_radius + 1):
        best = None
        for i in range(ci - r, ci + r + 1):
            for j in range(cj - r, cj + r + 1):
                if max(abs(i - ci), abs(j - cj)) != r or not grid.is_free_cell(i, j):
                    continue
                d = math.hypot(i - ci, j - cj)
                if best is None or d < best[0]:
                    best = (d, (i, j))
        if best:
            return best[1]
    return None


def astar(grid: GridMap, start: tuple[float, float], goal: tuple[float, float]) -> list[tuple[float, float]]:
    """Return the cell-by-cell path (as points) from start to goal."""
    s = grid.to_cell(*start)
    g = grid.to_cell(*goal)
    if not grid.is_free_cell(*g):
        raise PlanningError(f"goal {goal} is inside an obstacle or outside the flight boundary")
    prefix: list[tuple[float, float]] = []
    if not grid.is_free_cell(*s):
        free = _nearest_free_cell(grid, s)
        if free is None:
            raise PlanningError(f"start {start} is enclosed by obstacles")
        prefix = [start]
        s = free

    cols = grid.cols
    sidx, gidx = s[0] * cols + s[1], g[0] * cols + g[1]
    g_cost = {sidx: 0.0}
    parent = {sidx: -1}
    closed = set()

    def h(i: int, j: int) -> float:  # octile distance
        di, dj = abs(i - g[0]), abs(j - g[1])
        return (di + dj) + (SQRT2 - 2) * min(di, dj)

    heap = [(h(*s), 0, sidx)]
    counter = 0
    while heap:
        _, _, idx = heapq.heappop(heap)
        if idx in closed:
            continue
        if idx == gidx:
            break
        closed.add(idx)
        i, j = divmod(idx, cols)
        for di, dj, cost in _NEIGHBOURS:
            ni, nj = i + di, j + dj
            if not grid.is_free_cell(ni, nj):
                continue
            # no corner cutting: both orthogonal neighbours must be free for a diagonal move
            if di and dj and not (grid.is_free_cell(i + di, j) and grid.is_free_cell(i, j + dj)):
                continue
            nidx = ni * cols + nj
            new_cost = g_cost[idx] + cost
            if new_cost < g_cost.get(nidx, math.inf):
                g_cost[nidx] = new_cost
                parent[nidx] = idx
                counter += 1
                heapq.heappush(heap, (new_cost + h(ni, nj), counter, nidx))
    else:
        raise PlanningError(f"no path from {start} to {goal}")

    if gidx not in parent:
        raise PlanningError(f"no path from {start} to {goal}")
    cells = []
    idx = gidx
    while idx != -1:
        cells.append(grid.to_point(*divmod(idx, cols)))
        idx = parent[idx]
    cells.reverse()
    cells[-1] = (float(goal[0]), float(goal[1]))
    if not prefix:
        cells[0] = (float(start[0]), float(start[1]))
    return prefix + cells


def simplify(grid: GridMap, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Greedy line-of-sight shortcutting: keep only the corner waypoints."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        nxt = anchor + 1
        for k in range(len(path) - 1, anchor, -1):
            if grid.line_free(path[anchor], path[k]):
                nxt = k
                break
        out.append(path[nxt])
        anchor = nxt
    return out


def plan(cfg: dict, start: tuple[float, float], goal: tuple[float, float],
         extra_obstacles: list[Box] | None = None) -> list[tuple[float, float]]:
    """Plan a simplified collision-free waypoint list from start to goal."""
    grid = build_grid(cfg, extra_obstacles)
    return simplify(grid, astar(grid, start, goal))


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def path_length(path: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def point_along(path: list[tuple[float, float]], fraction: float) -> tuple[float, float]:
    """Point at `fraction` (0..1) of the total path length."""
    target = path_length(path) * min(max(fraction, 0.0), 1.0)
    for a, b in zip(path, path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if target <= seg and seg > 0:
            t = target / seg
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        target -= seg
    return path[-1]


def blockage_box(cfg: dict, route: list[tuple[float, float]]) -> Box:
    """The temporary blockage, centred on the route at the configured fraction."""
    b = cfg["blockage"]
    n, e = point_along(route, b["route_fraction"])
    return Box("blockage", round(n, 1), round(e, 1), b["size"], b["size"], b["height"])


def main() -> int:
    cfg = load_config(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG)
    home = (cfg["hospital"]["north"], cfg["hospital"]["east"])
    goal = (cfg["village"]["north"], cfg["village"]["east"])

    route = plan(cfg, home, goal)
    print(f"Outbound route ({path_length(route):.1f} m, {len(route)} waypoints):")
    for k, (n, e) in enumerate(route):
        print(f"  WP{k}: north={n:7.1f}  east={e:7.1f}")

    blk = blockage_box(cfg, route)
    print(f"\nBlockage placed at north={blk.north}, east={blk.east} ({blk.size_n} m square)")

    # Simulate detection: the drone stops `sensor_range` metres before the blockage.
    stop = point_along(route, max(cfg["blockage"]["route_fraction"] -
                                  cfg["blockage"]["sensor_range"] / path_length(route), 0.0))
    detour = plan(cfg, stop, goal, [blk])
    print(f"Re-planned route from {tuple(round(v, 1) for v in stop)} "
          f"({path_length(detour):.1f} m, {len(detour)} waypoints):")
    for k, (n, e) in enumerate(detour):
        print(f"  WP{k}: north={n:7.1f}  east={e:7.1f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import viz
        fig, ax = plt.subplots(figsize=(8, 8))
        viz.draw_static(ax, cfg)
        viz.draw_path(ax, route, "planned route (A*)", color="tab:blue")
        viz.draw_blockage(ax, blk, cfg["safety_margin"], active=True)
        viz.draw_path(ax, detour, "re-planned route", color="tab:green", style="--")
        ax.plot(stop[1], stop[0], "k^", ms=10, label="blockage detected here")
        ax.legend(loc="lower right", fontsize=8)
        ax.set_title("Medical delivery: A* route and blockage re-plan")
        out = os.path.join(ROOT, "docs", "plan_preview.png")
        fig.savefig(out, dpi=120, bbox_inches="tight")
        print(f"\nSaved preview image: {out}")
    except ImportError:
        print("\n(matplotlib not installed - skipping preview image)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
