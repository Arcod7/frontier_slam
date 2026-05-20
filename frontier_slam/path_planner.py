"""A* path planner on OccupancyGrid.

Dual-inflation strategy:
  HARD_INFLATION_M  — hard wall for A*: cells within this radius are completely
                       blocked (cost = inf).  Small enough that paths can still
                       pass through narrow corridors.
  INFLATION_M       — soft zone: cells between HARD and INFLATION_M receive a
                       high but finite cost (SOFT_COST), so A* routes around
                       them when a free path exists but can still pass through
                       when forced.

Unknown cells (-1) are treated as free (optimistic — correct for frontier
exploration).
"""
import heapq
import math

import numpy as np
from scipy.ndimage import binary_dilation

HARD_INFLATION_M = 0.20   # hard A* wall — must clear actual obstacles
INFLATION_M      = 0.75   # soft zone — high cost, last-resort passage
PLAN_INFLATION_M = 1.50   # planning margin — moderate cost, steers paths away from walls
SOFT_COST        = 8.0    # cost multiplier inside soft zone   (0.20 m … 0.75 m)
PLAN_COST        = 3.0    # cost multiplier inside planning zone (0.75 m … 1.50 m)
PAD_CELLS        = 5      # unknown-cell border added around the grid before planning


def _disk(radius: int) -> np.ndarray:
    r = radius
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y) <= r * r


def inflate_occupied(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Return a copy of grid with occupied (100) cells dilated by radius_cells."""
    if radius_cells <= 0:
        return grid
    result = grid.copy()
    result[binary_dilation(grid == 100, structure=_disk(radius_cells))] = 100
    return result


def _astar(cost_grid: np.ndarray, start: tuple, goal: tuple) -> list | None:
    """A* on a float cost_grid, 8-connectivity.

    cost_grid values:
      math.inf  — hard wall, never traversed
      SOFT_COST — soft zone, passable but expensive
      1.0       — free

    Returns list of (row, col) from start to goal inclusive, or None if
    no path exists.
    """
    if math.isinf(cost_grid[goal]):
        return None

    rows, cols = cost_grid.shape
    g_score = {start: 0.0}
    visited = set()
    counter = 0
    heap = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), counter, start, None)]
    parent = {}

    while heap:
        _, _, node, par = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)
        parent[node] = par

        if node == goal:
            path = []
            cur = goal
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return path

        r, c = node
        g = g_score[node]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                nb = (nr, nc)
                cell_cost = cost_grid[nb]
                if nb in visited or math.isinf(cell_cost):
                    continue
                step = (1.414 if dr != 0 and dc != 0 else 1.0) * cell_cost
                ng = g + step
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    counter += 1
                    h = math.hypot(goal[0] - nr, goal[1] - nc)
                    heapq.heappush(heap, (ng + h, counter, nb, node))
    return None


def _thin(cells: list, stride: int = 10) -> list:
    """Keep every stride-th cell plus the last."""
    if len(cells) <= 2:
        return list(cells)
    kept = cells[::stride]
    if kept[-1] != cells[-1]:
        kept.append(cells[-1])
    return kept


def plan_path(grid_msg, robot_xy: np.ndarray, goal_xy: np.ndarray,
              soft_inflation_m: float = INFLATION_M,
              hard_inflation_m: float = HARD_INFLATION_M,
              plan_inflation_m: float = PLAN_INFLATION_M) -> list:
    """Return world-frame (x, y) waypoints from robot to goal.

    Three-zone cost grid — A* prefers routes furthest from walls:
      hard zone     (d ≤ hard_inflation_m)              → inf   (blocked)
      soft zone     (hard_inflation_m < d ≤ soft_m)     → SOFT_COST  (8×)
      planning zone (soft_m < d ≤ plan_inflation_m)     → PLAN_COST  (3×)
      free / unknown                                     → 1.0

    Snapping start/goal uses the hard boundary only — goals inside the soft
    or planning zones are valid targets.
    """
    info = grid_msg.info
    res  = info.resolution
    w, h = info.width, info.height

    grid = np.array(grid_msg.data, dtype=np.int8).reshape(h, w)

    # Pad with unknown cells so A* can route near/past the observed map boundary.
    # Unknown cells (-1) have cost 1.0, so A* treats them as free space.
    grid = np.pad(grid, PAD_CELLS, constant_values=-1)
    h, w = grid.shape
    ox   = info.origin.position.x - PAD_CELLS * res
    oy   = info.origin.position.y - PAD_CELLS * res

    hard_r = max(1, int(round(hard_inflation_m / res)))
    soft_r = max(1, int(round(soft_inflation_m / res)))
    plan_r = max(1, int(round(plan_inflation_m / res)))
    hard_blocked = inflate_occupied(grid, hard_r) == 100
    soft_zone    = inflate_occupied(grid, soft_r) == 100
    plan_zone    = inflate_occupied(grid, plan_r) == 100

    cost_grid = np.ones((h, w), dtype=np.float64)
    cost_grid[plan_zone & ~soft_zone]    = PLAN_COST   # outer planning margin
    cost_grid[soft_zone & ~hard_blocked] = SOFT_COST   # inner soft zone
    cost_grid[hard_blocked]              = math.inf

    def to_cell(wx, wy):
        return int((wy - oy) / res), int((wx - ox) / res)   # (row, col)

    def to_world(r, c):
        return ox + (c + 0.5) * res, oy + (r + 0.5) * res

    sr, sc = to_cell(robot_xy[0], robot_xy[1])
    gr, gc = to_cell(goal_xy[0],  goal_xy[1])

    if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
        return []

    # Snap start/goal against hard boundary only — the goal may legitimately
    # sit in the soft zone.
    free = np.argwhere(~hard_blocked)
    if free.size == 0:
        return []

    def nearest_free(r, c):
        dists = np.hypot(free[:, 0] - r, free[:, 1] - c)
        i = int(np.argmin(dists))
        return int(free[i, 0]), int(free[i, 1])

    if hard_blocked[sr, sc]:
        sr, sc = nearest_free(sr, sc)
    if hard_blocked[gr, gc]:
        gr, gc = nearest_free(gr, gc)

    cells = _astar(cost_grid, (sr, sc), (gr, gc))
    if not cells:
        return []
    return [to_world(r, c) for r, c in _thin(cells)]
