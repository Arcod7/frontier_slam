# frontier_slam — Progress Log

Each entry records a change, its objective, and the **observed impact** once tested.
Entries are ordered chronologically. Mark impact as ✅ positive, ⚠️ mixed/partial,
❌ negative (reverted), or 🔲 not yet tested.

---

## Change 1 — Initial package creation

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`, `setup.py`, `setup.cfg`,
`package.xml`, `launch/frontier_slam.launch.py`

**Objective**: Create the baseline frontier-based exploration stack.
- `frontier_extractor`: reads `/projected_map`, detects free↔unknown boundaries,
  clusters with `scipy.ndimage.label`, publishes nearest centroid as `/frontier_slam/goal`.
- `waypoint_controller`: P-controller (surge + yaw + heave) toward the goal, publishes
  thruster setpoints to `/bluerov2/controller/thruster_setpoints_sim`.

**Key constants at creation**:
```
KP_YAW=0.4  KP_SURGE=0.25  KP_HEAVE=0.3
MAX_SURGE=0.40  GOAL_RADIUS=2.0  SCAN_YAW=0.08
MIN_CLUSTER_CELLS=5  UPDATE_HZ=0.5
```

**Observed impact**: 🔲 Launch failed — see Change 2.

---

## Change 2 — Fix missing `setup.cfg` (libexec directory)

**Date**: 2026-05-19  
**Files**: `setup.cfg` (created)

**Objective**: Fix launch error `"libexec directory does not exist"`. Without `setup.cfg`,
ament_python does not install console_scripts into `lib/<package_name>/`.

**Fix**:
```ini
[develop]
script_dir=$base/lib/frontier_slam
[install]
install_scripts=$base/lib/frontier_slam
```

**Observed impact**: ✅ Launch succeeded. Both nodes started.

---

## Change 3 — Add `MIN_EXPLORE_DIST` to skip nearby frontiers

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot was immediately entering "Goal reached — hovering" because the only
visible frontier was within `GOAL_RADIUS` of the start position.

**Fix**: Added `MIN_EXPLORE_DIST = 3.0 m` filter — any frontier closer than 3 m is
discarded before goal selection.

**Observed impact**: ✅ Robot started moving toward distant frontiers instead of hovering.

---

## Change 4 — Fix sinking during hover (depth hold)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot sank from depth 10 m to 18 m+ while hovering at "goal reached"
state. Zero thrust on vertical thrusters causes free-fall in NED frame (Z+ = down).

**Fix**: Always apply `heave_cmd = KP_HEAVE * (-dz)` — even when goal is reached.
Also apply a slow `SCAN_YAW` rotation to survey the area rather than sitting still.

Before:
```python
self._mix_and_send(0.0, 0.0, 0.0)
```
After:
```python
heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))
self._mix_and_send(0.0, self.SCAN_YAW, heave_cmd)
```

**Observed impact**: ✅ Robot held depth. Sinking eliminated.

---

## Change 5 — Replace hard heading threshold with cosine scaling

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Hard threshold at `HEADING_THRESH = 0.3 rad` caused bang-bang oscillation:
robot alternated between "spinning in place" and "surging forward", leading to a jerky
zigzag path and a wall collision.

**Fix**: Removed `HEADING_THRESH` entirely. Replaced with smooth cosine scaling:
```python
surge_raw = KP_SURGE * dist_xy * max(0.0, math.cos(heading_err))
```
Surge is maximum when perfectly aligned (cos=1), drops to zero at 90° misalignment,
and naturally produces a curved approach path.

**Observed impact**: ✅ Smooth curved approach. No more oscillation. Wall collision eliminated.

---

## Change 6 — Add goal hysteresis (`_committed` + `SWITCH_HYSTERESIS`)

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot oscillated between two equidistant frontier centroids every 2 s,
flipping the goal on each update cycle. Wasted time, caused erratic heading.

**Fix**: Added `_committed` goal state. A new candidate only replaces the current goal
if it is more than `SWITCH_HYSTERESIS = 3.0 m` closer than the committed one. If the
committed goal's cluster has drifted (map update), track the drift by updating
`_committed` without resetting the timer.

**Observed impact**: ✅ Goal oscillation eliminated. Robot commits to one direction.

---

## Change 7 — Slow down yaw and scan (10×)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Heading changes were abrupt and visually jarring. User requested 10× slower
turning for smoother behaviour.

**Fix**:
```python
KP_YAW   = 0.4  → 0.04
SCAN_YAW = 0.08 → 0.008
```

**Observed impact**: ✅ Smooth, gradual heading changes. User confirmed improvement.

---

## Change 8 — Forward obstacle avoidance (depth camera proximity)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot collided with a wall while surging toward a frontier behind it.
Add reactive braking using the forward depth camera.

**Fix**: Subscribe to `/sensor_msgs/image_depth` (32FC1). Parse the central 40% horizontal
strip, take the minimum finite range, and scale surge linearly:
- `obs_factor = 1.0` at ≥ 2.5 m → full surge allowed
- `obs_factor = 0.0` at ≤ 0.8 m → surge cut to zero

```python
obs_factor = min(1.0, max(0.0,
    (self._min_front_dist - OBSTACLE_STOP_DIST) /
    (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
))
surge_cmd = clip(surge_raw * obs_factor, 0.0, MAX_SURGE)
```

**Observed impact** (session 1): ⚠️ No wall collisions observed — avoidance works.
However, the robot repeatedly gets [BLOCKED] for 20-30 s when turning away from structure
corner near (14.8, 2.5): heading error starts at 120-150°, forward camera sees wall at
0.2-0.7 m during the turn, so surge stays zero while yaw slowly clears the obstacle.
Not dangerous but wastes significant exploration time each visit.

**Known limitation**: Only the forward camera is used. The robot has no lateral or rear
awareness — wall strikes from the side remain possible. See FutureWork §3.

---

## Change 9 — Stuck detection + goal blacklisting

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot was committed to goal (12.0, −7.4, 8.5) for 20+ seconds with no
progress — likely blocked by a wall. No mechanism existed to abandon unreachable goals.

**Fix**: Track distance to committed goal at commit time (`_committed_dist`). On each
update cycle check:
```
elapsed > STUCK_TIMEOUT (15 s)  AND  progress < STUCK_MIN_PROGRESS (0.5 m)
```
If stuck: WARN, add goal to `_blacklist` for `BLACKLIST_DURATION = 30 s`, clear
`_committed` so the next best candidate is selected.

**New constants**:
```
STUCK_TIMEOUT      = 15.0 s
STUCK_MIN_PROGRESS = 0.5 m
BLACKLIST_DURATION = 30.0 s
GOAL_VANISH_DIST   = 3.0 m   (radius for blacklist proximity check)
```

**Observed impact** (session 1): ✅ Triggered correctly 3 times:
- `(12.0,-7.4)` after 16s, 0.00m progress
- `(9.3,-9.1)` after 16s, 0.15m progress
- `(11.9,-11.1)` after 16s, 0.06m progress

Side effect: all 3 blacklisted simultaneously → "All candidates blacklisted" for ~14s
(waiting for first 30s timer to expire). Robot holds position during this window.

---

## Change 10 — Rich diagnostic logging

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Previous logs gave no visibility into why the robot was stuck. Added
structured per-cycle logs to allow post-hoc diagnosis from log output alone.

**frontier_extractor** (every update cycle, ~2 s):
```
robot=(x,y,z)  goal=(gx,gy)  dist=X.Xm  clusters=N  stuck=X%  blacklist=N
```
`stuck%` = percentage of `STUCK_TIMEOUT` elapsed on current goal.

**waypoint_controller** (throttled 2 s):
```
pos=(x,y,z) goal=(gx,gy,gz) dist=X.Xm  hdg_err=±Xdeg  surge=X.XX  yaw=±X.XXX  heave=±X.XX  obs=X.Xm  [BLOCKED]
```
`[BLOCKED]` tag appears when `obs_factor < 0.05`.

**Observed impact** (session 1): ✅ Logs were essential for diagnosing all issues in session 1.
Identified: initial stuck (0.00m progress), BLOCKED pattern at (14.8, 2.5) corner,
back-and-forth loop between two frontiers, Z drift.

---

## Change 11 — Automatic CSV session logging

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Manual log pasting is tedious and lossy. Both nodes now write structured
CSV files automatically to `logs/` at startup, enabling session-to-session comparison.

**Format**: Two files per session, named by wall-clock start time:
- `YYYY-MM-DD_HH-MM-SS_extractor.csv` — written at 0.5 Hz (every update cycle)
- `YYYY-MM-DD_HH-MM-SS_controller.csv` — written at 1 Hz (every 10th control tick)

**extractor columns**: `t_ros, rx, ry, rz, gx, gy, dist_m, clusters, stuck_pct, blacklist_n, event`  
**controller columns**: `t_ros, rx, ry, rz, gx, gy, gz, dist_m, hdg_err_deg, surge, yaw_cmd, heave, obs_m, blocked, event`

**event values**:
- extractor: `""` (normal), `STUCK_BLACKLIST`, `ALL_BLACKLISTED`
- controller: `""` (normal), `BLOCKED`, `GOAL_REACHED`, `SCAN`

Log path is derived from `__file__` via `os.path.realpath()` so it always resolves
to the source `logs/` directory regardless of how the package is installed.

**Observed impact**: 🔲 Not yet tested.

---

## Current parameter snapshot

### frontier_extractor.py
| Parameter | Value | Purpose |
|---|---|---|
| `MIN_CLUSTER_CELLS` | 5 | Discard tiny frontier clusters (noise) |
| `MIN_EXPLORE_DIST` | 3.0 m | Skip frontiers the robot is already at |
| `SWITCH_HYSTERESIS` | 3.0 m | Only switch goals if new one is this much closer |
| `GOAL_VANISH_DIST` | 3.0 m | Committed goal "gone" if no cluster within this radius |
| `STUCK_TIMEOUT` | 15.0 s | Abandon goal after this long with no progress |
| `STUCK_MIN_PROGRESS` | 0.5 m | Minimum distance closed to not be "stuck" |
| `BLACKLIST_DURATION` | 30.0 s | Blacklist duration after abandoning a goal |
| `UPDATE_HZ` | 0.5 Hz | Frontier re-evaluation rate |

### waypoint_controller.py
| Parameter | Value | Purpose |
|---|---|---|
| `KP_YAW` | 0.04 | Heading P-gain |
| `KP_SURGE` | 0.25 | Forward speed P-gain |
| `KP_HEAVE` | 0.3 | Vertical speed P-gain |
| `MAX_SURGE` | 0.40 | Surge clamp |
| `GOAL_RADIUS` | 2.0 m | "Goal reached" threshold |
| `SCAN_YAW` | 0.008 | Slow yaw rotation when hovering |
| `OBSTACLE_SLOW_DIST` | 2.5 m | Start reducing surge below this |
| `OBSTACLE_STOP_DIST` | 0.8 m | Cut surge to zero below this |
| `CTRL_HZ` | 10.0 Hz | Control loop rate |

---

## Log sessions

Raw logs go in `logs/`. From Change 11 onward, CSV files are generated automatically.

| File | Date | Summary |
|---|---|---|
| `2026-05-19_13-46-59_session1_raw.log` | 2026-05-19 | Pre-CSV manual log. 3 stuck detections. Loop established between (14.7,2.2) and (10,9). Z drift 8.5→13.8m. |

---

## Session 1 — Findings (2026-05-19)

**Duration**: ~4 min of captured log  
**File**: `logs/2026-05-19_13-46-59_session1_raw.log`

### What worked
- **Stuck detection** ✅: All 3 blacklisting events fired correctly and enabled goal switching
- **Obstacle avoidance** ✅: No wall collisions; [BLOCKED] correctly prevented surge near walls
- **Goal reached** ✅: Robot successfully navigated to and reached multiple frontiers after initial stucks

### Issues observed

**1. BLOCKED delay at structure corner (~14.8, 2.5)**
After reaching (14.7, 2.2), the robot needs to turn 120-150° to its next goal. During
this turn the forward camera faces the wall (obs=0.2-0.7m), blocking surge for ~20s
per visit. The robot eventually yaws clear and surges — no crash — but wastes time.
Root cause: slow KP_YAW=0.04 combined with OBSTACLE_STOP_DIST cutting surge during turn.
Potential fix: increase yaw rate when BLOCKED (e.g. `KP_YAW_BLOCKED = 0.15`).

**2. Back-and-forth loop between two frontiers**
After initial stucks cleared, robot locked into a ~60s cycle between (14.7, 2.2) and
(10.0-10.3, 9.x). Nearest-first selection always picks these same two clusters because
they remain visible frontier boundaries throughout the run.
Root cause: nearest-frontier scoring ignores cluster size.
Potential fix: score by `cluster_size / distance` (FutureWork §1) — larger distant
clusters beat small nearby ones, breaking the loop.

**3. Z drift**
Robot depth increases from 8.5m to 13.8m+ over ~4 minutes. Goal Z is set to robot's Z
at publish time (frontier_extractor), but robot drifts down between updates (2s cycle).
The small positive heave commands (+0.00 to +0.06) are insufficient to counteract
persistent downward drift — possibly buoyancy simulation or integration error.
Not critical for current testing but will become a problem in deeper environments.
