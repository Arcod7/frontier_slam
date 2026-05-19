# frontier_slam

**Frontier-based exploration baseline for underwater autonomous inspection.**

This package implements a classical frontier SLAM stack for the BlueROV2 running in the
[Stonefish](https://github.com/patrykcieslak/stonefish) underwater simulator under ROS 2 Jazzy.
Its primary purpose is to establish a **performance baseline** against which more sophisticated
exploration algorithms — in particular the saliency-driven revisit policy from Suresh et al.
(ICRA 2020) — will be compared.

## Goal

Autonomous 3-D inspection of underwater structures (offshore platforms, pipelines) requires
the robot to efficiently cover unknown space while maintaining an accurate map for loop
closure. Greedy frontier exploration is the canonical baseline: it is simple, well-understood,
and exposes the fundamental trade-offs that more complex methods aim to improve.

By measuring how well the frontier baseline explores the structure (coverage %, path length,
time to map closure, number of stuck events), we get a concrete reference point. Any proposed
improvement — information-gain scoring, saliency-driven revisits, path planning — must
outperform this baseline on those same metrics to be considered worthwhile.

## How it works

```
/projected_map (OccupancyGrid)           /sensor_msgs/image_depth
        │                                        │
        ▼                                        │
frontier_extractor ──────────────────────────── │
  - detect free/unknown boundaries              │
  - cluster frontiers (scipy.ndimage)           │
  - pick goal: nearest cluster > 3 m            │
  - hysteresis, stuck detection, blacklisting   │
        │                                        │
        │ /frontier_slam/goal (PointStamped)     │
        ▼                                        ▼
waypoint_controller
  - XY: yaw toward goal, surge when aligned
  - depth hold: locked setpoint from first odom
  - obstacle brake: scale surge by forward range
        │
        ▼
/bluerov2/controller/thruster_setpoints_sim
```

## Package structure

```
frontier_slam/
├── frontier_extractor.py   # ROS node: map → goal publisher
├── waypoint_controller.py  # ROS node: goal → thruster setpoints
├── frontier_detection.py   # pure: OccupancyGrid → list[Cluster]
├── goal_manager.py         # state: hysteresis, stuck detection, blacklist
├── control_utils.py        # pure: thruster mixing, yaw helpers
├── session_log.py          # shared: timestamped CSV logging
├── launch/
│   └── frontier_slam.launch.py
├── logs/                   # auto-generated per-session CSVs (gitignored)
├── Progress.md             # change log with observed impact per session
└── FutureWork.md           # candidate improvements and their rationale
```

## ROS interface

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/projected_map` | `OccupancyGrid` | 2-D OctoMap projection |
| in | `/StoneFish/Odometry` | `Odometry` | robot pose + velocity |
| in | `/sensor_msgs/image_depth` | `Image` (32FC1) | forward depth camera |
| out | `/frontier_slam/goal` | `PointStamped` | current exploration goal |
| out | `/frontier_slam/frontiers` | `MarkerArray` | RViz visualization |
| out | `/bluerov2/controller/thruster_setpoints_sim` | `Float64MultiArray` | 6 thruster commands |

## Build and run

The workspace builds inside a `ros2-jazzy` distrobox container (see `slam_ws/src/CLAUDE.md`
for the full environment setup).

```bash
distrobox enter ros2-jazzy
cd ~/delivery/slam_ws
colcon build --symlink-install
source install/setup.zsh
ros2 launch frontier_slam frontier_slam.launch.py
```

The simulation must be running first (steps 1-3 from `CLAUDE.md`).

## Session logs

Both nodes write CSV files to `logs/` at startup:

- `YYYY-MM-DD_HH-MM-SS_extractor.csv` — frontier selection at 0.5 Hz
- `YYYY-MM-DD_HH-MM-SS_controller.csv` — control state at 1 Hz

See `Progress.md` for a history of changes and their measured impact on robot behaviour.

## Roadmap

The intended progression is:

1. **This package** — greedy nearest-frontier baseline, P-controller navigation
2. Information-gain frontier scoring (`cluster_size / distance`)
3. A\* path planning on the occupancy grid (eliminate wall-following failures)
4. Saliency-driven revisit policy (Suresh et al. 2020) — the core thesis contribution

See `FutureWork.md` for detailed notes on each step.

## Reference

> Suresh, S., Yogamani, S., & Ganesan, K. (2020). *Active SLAM using 3D Submap Saliency
> for Underwater Volumetric Exploration*. IEEE ICRA 2020.
