"""Pure functions for detecting frontier clusters in an OccupancyGrid.

A frontier cell is OCCUPIED (=100) and directly adjacent to UNKNOWN (<0). These
wall-surface cells are stable targets: they don't recede as the robot approaches,
and navigating near them lets the sonar illuminate the unknown space beyond the wall
from a new angle. The 2D projected map from OctoMap satisfies this convention.
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation, label


@dataclass
class Cluster:
    wx: float          # world-frame x of centroid (m)
    wy: float          # world-frame y of centroid (m)
    size: int          # number of cells in the cluster
    distance: float = 0.0  # to a reference point — filled in by the caller


def find_frontier_clusters(grid_msg, min_cluster_cells: int = 5) -> list:
    """Return clusters of frontier cells in the OccupancyGrid.

    The returned distance field is left at 0.0; the caller is expected to fill
    it relative to whatever reference (usually the robot pose) it cares about.
    """
    res = grid_msg.info.resolution
    ox  = grid_msg.info.origin.position.x
    oy  = grid_msg.info.origin.position.y
    w, h = grid_msg.info.width, grid_msg.info.height

    grid     = np.array(grid_msg.data, dtype=np.int8).reshape(h, w)
    occupied = (grid == 100)
    unknown  = (grid < 0)
    frontier = occupied & binary_dilation(unknown)

    labeled, n = label(frontier)
    if n == 0:
        return []

    clusters = []
    for i in range(1, n + 1):
        rows, cols = np.where(labeled == i)
        if len(rows) < min_cluster_cells:
            continue
        wx = ox + cols.mean() * res
        wy = oy + rows.mean() * res
        clusters.append(Cluster(wx=float(wx), wy=float(wy), size=len(rows)))
    return clusters
