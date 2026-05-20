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

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): both CSV files created at startup,
both nodes wrote correct data for 82s. Float formatting, column names, and flush-on-write
all confirmed. Controller CSV `depth_err_m` column confirms the refactored schema was live.

---

## Change 12 — Depth-hold uses a fixed setpoint (Z drift fix)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `frontier_extractor.py`

**Objective**: Session 1 logs showed the robot sinking from 8.5 m to 13.8 m over 4 minutes
even though every heave command was small (+0.00 to +0.06). The cause was a feedback
loop in the goal-Z plumbing:

```
extractor: gz = robot.z       ← uses CURRENT robot z each cycle
controller: heave = KP * (-(gz - pose.z))
```

When the robot drifted down between updates (2 s), the next frontier message brought a
new, deeper `gz`. The depth error never grew large enough to trigger meaningful heave,
so each cycle the controller accepted the drifted depth as the new target.

**Fix**:
- `WaypointController` now owns the depth setpoint. It is captured once on the first
  odometry callback (`self._depth_setpoint = pose.z`) and held for the lifetime of the
  node. The control law became `heave = KP_HEAVE * (pose.z - setpoint)`.
- The goal's Z field is now ignored by the controller. The extractor still publishes
  `gz = robot.z` but only so the RViz marker sits at the right height — it does not
  influence the depth controller.
- `KP_HEAVE` bumped from 0.30 → 0.40 to give a bit more authority against any
  persistent buoyancy bias.

**Observed impact**: ❌ Sign inverted — robot sank rapidly. `KP_HEAVE * (pose.z - setpoint)`
gives **positive** heave when robot is too deep, but positive = DOWN in Stonefish.
The robot fell faster than without any depth control. Fixed in Change 14.

---

## Change 13 — Refactor: split into focused modules

**Date**: 2026-05-19  
**Files**: new `session_log.py`, `frontier_detection.py`, `goal_manager.py`,
`control_utils.py`; rewritten `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Both node files had grown to mix several unrelated concerns (frontier
detection + goal state + viz + CSV logging in one; thruster mixing + control law +
CSV logging in the other). The single update/loop methods were 80+ lines.

**Structure**:
```
frontier_slam/
├── frontier_extractor.py     # node: orchestration, ROS I/O, marker viz
├── waypoint_controller.py    # node: orchestration, ROS I/O, control loop
├── frontier_detection.py     # pure: OccupancyGrid → list[Cluster]
├── goal_manager.py           # state: hysteresis, stuck detection, blacklist
├── control_utils.py          # pure: yaw_from_quat, wrap_angle, mix_thrusters
└── session_log.py            # shared: CSV opener + float-formatting writer
```

**Behavioural guarantees**: refactor only — no algorithmic changes alongside Change 12.
Same constants, same control law, same selection rules. Identical CSV schemas
(controller gained one column: `depth_err_m`).

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): all modules imported correctly,
both nodes started, frontier detection ran (7 clusters at t=0), GoalManager selected and
switched goals, stuck detection was not triggered (no regression). No behavioural
differences from pre-refactor sessions observed.

---

## Change 14 — Fix heave sign (Stonefish convention: negative = UP)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Change 12 introduced `heave = KP_HEAVE * (pose.z - setpoint)`. In NED,
`pose.z - setpoint > 0` when the robot is too deep — the correct response is upward
thrust. In the Stonefish simulation **negative heave drives upward thrust**, so the
formula produced strong downward thrust, accelerating the sink.

**Fix**: Negate the error term:
```python
# Before (wrong — positive when deep = pushes DOWN):
return float(np.clip(self.KP_HEAVE * depth_err, -1.0, 1.0))

# After (correct — negative when deep = pushes UP in Stonefish):
return float(np.clip(-self.KP_HEAVE * depth_err, -1.0, 1.0))
```

The docstring is updated to state "Negative output = upward thrust in Stonefish" to
prevent future sign confusion.

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): depth setpoint locked at 8.14m
on first odom. Z stayed in [8.12, 8.25]m over 82s (max error 0.11m). Compare to Session 1
where z drifted 5.3m in 4 minutes with no depth control. `depth_err_m` column confirms
the controller was actively correcting (heave between −0.01 and −0.06 throughout).

---

## Change 15 — Goal timeout: clear stale goal and enter scan mode

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 2 showed the robot frozen in GOAL_REACHED for 63s after frontiers
ran dry. The controller held `_goal` forever; SCAN mode (`_goal is None`) was never entered.

**Fix**: Track when GOAL_REACHED state was first entered. If no new goal arrives within
`GOAL_REACHED_TIMEOUT = 10.0 s`, clear `_goal` and log. Next tick enters SCAN mode
(slow yaw rotation) which may help the robot find new frontiers by rotating into new
areas. Timer resets whenever a fresh goal arrives.

```python
if now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
    self._goal = None          # → next tick: SCAN mode
    self._goal_reached_at = None
```

**Observed impact**: ✅ Session 3 (`2026-05-19_15-05-00`): coverage columns present in
extractor CSV. Confirmed growth from 8514 → 9756 mapped cells (+14.6%) in 12s of
exploration. Largest gain (+556 cells) when robot was closest to wall (obs=0.23m).
Coverage stalled once robot moved into open water — useful signal for future sessions.

---

## Change 17 — Restore SCAN_YAW to 0.08 (split scan speed from nav yaw)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 3 showed the robot frozen at (14.44, −4.06) in SCAN mode for 59s
without finding any new frontiers. `SCAN_YAW=0.008` was reduced 10× in Change 7 alongside
`KP_YAW` to smooth navigation turns. That reduction was appropriate for navigation but
made scanning nearly useless: ~13 minutes for a full rotation.

**Fix**: Restore `SCAN_YAW = 0.08` (its pre-Change-7 value). `KP_YAW` stays at 0.04 —
the two constants now serve different purposes and should be tuned independently.

At 0.08 the robot completes a meaningful scan arc in a few seconds, giving the extractor
a chance to see new frontier cells as the camera sweeps past unexplored structure.

**Observed impact**: ✅ Session 3 (`2026-05-19_15-05-00`): GOAL_REACHED entered at t=519,
goal cleared and SCAN entered at t=530 (11s later). Controller CSV shows clean transition:
rows switch from `GOAL_REACHED` with valid goal columns to `SCAN` with empty goal columns.
Timer reset on new goal confirmed by clean re-acquisition at start of session.

---

## Change 19 — Frontier scoring: prefer large clusters over nearest cluster

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 7 showed the robot looping for 231s between two equidistant frontier
clusters (A and B at opposite corridor ends), gaining only 74 new cells. The root cause:
`candidates.sort(key=lambda c: c.distance)` gives equal weight to any two clusters at the
same distance regardless of size. When the large unexplored installation is 18m away and a
small 4-cell boundary patch is 15m away, nearest-first picks the patch every time.

**Fix**: Change sort key to `c.distance / c.size` (ascending = best first). A cluster of
50 cells at 18m scores 0.36; a cluster of 4 cells at 15m scores 3.75. The robot now
consistently heads toward the information-rich region.

Side effect: noise immunity. OccupancyGrid noise creates phantom clusters of 5–8 cells
in already-explored areas. These score poorly vs genuine large frontiers — the robot
ignores them without any additional filtering.

The mid-journey hysteresis (`switch_hysteresis = 3.0m`) is intentionally kept distance-based:
it prevents abandoning a large distant goal for a small nearby distraction mid-journey.

**Observed impact**: ✅ Session 8 (`2026-05-19_16-15-40`): robot traveled south to y=−14
(session 7 reached only y=−4). 16,100 new cells in 311s vs 11,803 in 360s — +58% cells/minute.
Odometer dropped from 262m to 149m — robot is exploring smarter. The large unexplored
installation was reached and partially mapped. New failure mode exposed: after 3 STUCK events
in the obstacle-dense south zone, only 4 frontier clusters remained and exploration stalled.

---

## Change 18 — Obstacle escape: forced yaw when BLOCKED + position-stuck detection

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Sessions 4–6 exposed two distinct failure modes that the previous obstacle
avoidance could not handle:

1. **Aligned BLOCKED** (`obs < STOP_DIST`, `heading_err ≈ 0`): robot is aimed directly at
   a pillar; both surge and yaw drop to ~0. No escape maneuver.
2. **Off-axis physical stuck** (`obs ≈ 2.4m`, `blocked=False`): depth camera sees 2.4m of
   clear water straight ahead, but the robot's body is pressing against a wall or pillar
   that is slightly off-axis from the central 40% strip. Controller applies full `surge=0.40`
   indefinitely; robot barely moves (position oscillates ±0.03m for 35+ seconds).

**Fix — two mechanisms added:**

*A. Escape yaw on BLOCKED*: When `obs < STOP_DIST` (sensor-blocked), override the
goal-tracking `yaw_cmd` with `±ESCAPE_YAW = 0.40` in the direction of the heading error.
This ensures the robot always pivots away from a head-on obstacle regardless of heading_err magnitude.

*B. Position-stuck detection*: If `surge ≥ STUCK_SURGE_MIN = 0.15` continuously for
`STUCK_WINDOW = 5.0s` without moving more than `STUCK_MOVE_MIN = 0.25m`, the controller
logs `CTRL_STUCK`, sets `_escape_until = now + ESCAPE_DURATION`, and spins at `ESCAPE_YAW`
for `ESCAPE_DURATION = 4.0s` before resuming normal drive. This catches off-axis physical
blockages that the depth camera cannot see. The stuck reference is also reset whenever a
new goal arrives.

New CSV `event` values: `STUCK_ESCAPE` (spinning during escape), `CTRL_STUCK` (trigger row).

New constants: `ESCAPE_YAW=0.40`, `ESCAPE_DURATION=4.0s`, `STUCK_SURGE_MIN=0.15`,
`STUCK_WINDOW=5.0s`, `STUCK_MOVE_MIN=0.25m`.

**Observed impact**: ✅ Session 8 (`2026-05-19_16-15-40`): all BLOCKED events resolve in
1–3 ticks (max consecutive run 15 ticks at one south-zone pillar). No frozen BLOCKED events
like sessions 4–5 (80+ seconds). CTRL_STUCK did not fire — a gap was found: position-stuck
requires `surge≥0.15` which is never true when BLOCKED (obstacle factor kills surge). Robot
can spin indefinitely in BLOCKED state without triggering the escape timer. Fix tracked as
future work (add BLOCKED-duration counter separate from surge-based detection).

---

## Change 16 — Map coverage metrics in extractor CSV

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: The extractor CSV gave no visibility into how much of the map was being
built over time. Without coverage numbers it is impossible to compare exploration
efficiency across sessions or algorithm variants.

**Fix**: Added `_map_stats()` which counts OccupancyGrid cell states on each update
cycle. Three new columns added to `extractor.csv`:

| Column | Meaning |
|---|---|
| `free_cells` | Cells with value 0 (navigable space confirmed) |
| `occ_cells` | Cells with value 100 (obstacle confirmed) |
| `mapped_cells` | `free + occ` — total cells with known state |

`mapped_cells` is the primary coverage metric: it grows as the robot explores and
plateaus when exploration stalls. Also printed in the extractor's per-cycle ROS log line.

**Observed impact**: ✅ Session 7 (`2026-05-19_15-55-06`): `mapped_cells` tracked continuously
from 15,348 at session start to 27,151 at plateau. Growth rate visible in CSV (fast early,
then near-zero after t≈135s). Used to confirm the back-and-forth loop in phase 4.

---

## Change 20 — Lateral obstacle readings (left / right strips)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `tools/analyze_session.py` (new)

**Objective**: The forward depth camera previously used only the central 40% column strip
(30%–70%) to measure obstacle distance. This left the 30% on each side unseen — a robot
pressing sideways against a lateral wall could still show `obs_m = inf` and full surge.
Session 8 showed multiple south-zone BLOCKED chains that the forward camera alone couldn't
fully characterise.

Additionally: created `tools/analyze_session.py` as a standalone analysis script intended
to evolve with the system. Produces per-session reports and two-session comparisons from
controller + extractor CSV files.

**Fix — lateral strips**:
The single `_depth_cb` extraction is split into three non-overlapping strips:

| Strip | Columns | State variable |
|---|---|---|
| Left  | 0%–30%  | `_min_left_dist` |
| Centre| 30%–70% | `_min_front_dist` (unchanged) |
| Right | 70%–100%| `_min_right_dist` |

Two new CSV columns: `obs_left_m`, `obs_right_m` inserted between `obs_m` and `blocked`.
The obstacle factor and BLOCKED decision still use only `_min_front_dist` — lateral values
are logged for analysis. Using them for active steering is future work (e.g. soft wall-following
or lateral escape direction preference).

**New CSV columns (controller)**:
```
… obs_m, obs_left_m, obs_right_m, blocked, event
```

**tools/analyze_session.py**:
- Auto-detects latest session or accepts a prefix argument
- `--compare A B`: side-by-side metric table for two sessions
- `--list`: enumerate all sessions with duration
- Sections: Overview, Coverage (with per-quarter bar chart), Navigation,
  Goal behaviour, Obstacle/BLOCKED, Depth control
- Reads `obs_left_m` / `obs_right_m` if present in log (graceful fallback for older logs)

**Observed impact**: 🔲 Not yet tested.

---

## Change 21 — BLOCKED direction hysteresis + timeout escape

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 8 showed two BLOCKED-state problems:

1. **Direction jerk**: when `heading_err` alternates sign on consecutive ticks (robot spinning
   near a wall at a shallow angle), `yaw_cmd` flipped between +0.40 and −0.40 every second.
   This looks like rapid jerking to the observer and produces no net rotation.
2. **Indefinite BLOCKED stall**: position-stuck detection requires `surge≥0.15`, which is
   always false when BLOCKED (obstacle factor kills surge). The robot could spin in BLOCKED
   state for 15+ seconds with no escape trigger.

**Fix — direction hysteresis**: Track `_escape_yaw_dir` (+1 or -1) and `_escape_dir_flip_t`.
Only flip spin direction after `BLOCKED_DIR_MIN_DURATION = 2.0s` in the current direction.
This commits the robot to one rotation sense long enough to actually clear the obstacle.

**Fix — BLOCKED timeout**: Track `_blocked_since`. If continuously BLOCKED for
`BLOCKED_ESCAPE_TIMEOUT = 5.0s`, trigger the same committed 4s escape spin as `CTRL_STUCK`.
New CSV event: `BLOCKED_TIMEOUT`.

Both `_blocked_since` and `_escape_yaw_dir` reset on new goal arrival (may be in clear water).
`_blocked_since` also resets after each committed escape spin completes.

New constants: `BLOCKED_ESCAPE_TIMEOUT=5.0s`, `BLOCKED_DIR_MIN_DURATION=2.0s`.

**Observed impact**: 🔲 Not yet tested.

---

## Change 22 — Goal manager: minimum commit duration + longer blacklist

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 8 exposed two related failure modes in `_pick_with_hysteresis`:

1. **17m U-turn oscillation**: after a STUCK_BLACKLIST, the only candidate was sometimes
   (12.90, 3.00) at 17m north. Within seconds of heading that way, a southern cluster at
   ~3m re-appeared with a competitive score, causing an immediate reversal. The 3m
   `switch_hysteresis` is too small when the committed goal is 17m away.
2. **30s blacklist too short**: goal (6.6, −12.5) was blacklisted at t=140s and t=214s —
   the 30s expired and the robot immediately retried the same unreachable area.

**Fix — `min_commit_duration = 10.0s`**: Score-based switches are blocked for the first
10s after any new commitment. The goal-vanished path (committed cluster disappears from the
map) is still allowed to switch immediately — that reflects a genuine map change.

**Fix — `blacklist_duration` default: 30s → 60s**: Doubles the cooling-off period.
A goal that couldn't be reached in 15s of trying is unlikely to become reachable in the
next 30s; 60s gives the map more time to update or the robot's approach angle to change.

Both parameters are constructor arguments with sensible defaults — no call-site changes needed.

**Observed impact**: 🔲 Not yet tested.

---

## Change 23 — Fix CTRL_STUCK timer reset + reduce min_commit_duration

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `goal_manager.py`

**Objective**: Session 9 analysis revealed two bugs that together prevented any recovery
from a 111-second freeze near (7.6, −4.1):

1. **CTRL_STUCK timer reset bug** (`waypoint_controller.py`): `_goal_cb` unconditionally
   reset `_stuck_ref_pos = None` on every call. The frontier extractor republishes at
   0.5 Hz (every 2s), so the stuck timer was reset every 2s. With `STUCK_WINDOW = 5.0s`,
   the 5s accumulation window could never be reached — the timer was always cleared before
   it matured. The GoalManager STUCK mechanism also failed: the robot had made 7.4m of
   progress on first approach (enough to clear the 0.5m threshold), so the
   `committed_dist - cur_dist < STUCK_MIN_PROGRESS` check permanently returned "not stuck"
   even after the robot froze.

2. **min_commit_duration = 10s too conservative** (`goal_manager.py`): Session 9 produced
   only 7 goal commits vs 35 in session 8. The robot stayed locked on distant goals even
   when large newly-discovered frontiers appeared closer. The robot never explored the
   "other side of the boat" or the left building's far side — likely because min_commit_duration
   suppressed the switches that would have redirected it.

**Fix 1 — `_goal_cb` goal-change threshold**:
```python
goal_changed = (self._goal is None or
                np.hypot(new_goal[0] - self._goal[0],
                         new_goal[1] - self._goal[1]) > 1.0)
self._goal = new_goal
self._goal_reached_at = None
if goal_changed:
    self._stuck_ref_pos = None
    self._stuck_ref_t   = None
    self._blocked_since = None
```
The extractor republishing the same frontier (±0.1m drift) no longer resets the stuck
window. `_goal_reached_at` is still reset unconditionally — always correct on any goal update.

**Fix 2 — `min_commit_duration` 10.0s → 5.0s**:
Halving the minimum commit time lets the robot respond to better frontiers discovered en
route (e.g. a large cluster that becomes visible as the map grows) while still blocking
the rapid flip-flopping that triggered the session 8 U-turn oscillation.
The 5s window keeps the anti-oscillation benefit for transitions under ~5m radius.

**Observed impact**: 🔲 Not yet tested.

---

## Session 9 — Findings (2026-05-19)

**Duration**: 182s  
**Files**: `logs/2026-05-19_16-50-54_*.csv`

### Summary

Mixed result. The robot explored well in the first 100s, mapping 23,127 cells (session 8
reached 28,139 in 311s, but session 9 ended earlier). Coverage stalled completely from
t≈120s to end: robot frozen at (7.6, −4.1) for 62s with goal (6.78, −9.38) visible but
unreachable — neither CTRL_STUCK nor GoalManager STUCK fired.

Goal commits: only 7 vs 35 in session 8 — `min_commit_duration=10s` was too conservative.
The robot missed the "other side of the boat" and the far side of the left building.

### What worked

- **BLOCKED_TIMEOUT (Ch21)** ✅: no evidence of long BLOCKED stalls; wall contacts were
  brief. Direction hysteresis appears to have helped.
- **Frontier scoring (Ch19)** ✅: robot still reached interesting deep zones (south
  corridor, y≈−9).
- **Depth hold** ✅: z stayed near 8.2m throughout.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active south exploration | 0–60s | 12,897 → 17,355 | Rapid exploration south, 4,458 new cells |
| 2 — North sweep attempt | 60–100s | 17,355 → 22,122 | Goal switched to (9.6, 3.85), 4,767 new cells |
| 3 — Approach & freeze | 100–120s | 22,122 → 23,127 | Goal (6.78, −9.38), robot approached to 5.3m then froze |
| 4 — Full freeze | 120–182s | 23,127 → 23,127 | Zero new cells, stuck at (7.6,−4.1), 0 CTRL_STUCK events |

### Root causes of session 9 freeze

1. **CTRL_STUCK timer reset**: `_goal_cb` resets `_stuck_ref_pos` every 2s from the
   extractor republish — the 5s CTRL_STUCK window can never accumulate. Robot frozen
   for 62s with surge≥0.15 but never triggering escape. (**Fixed in Ch23**)

2. **GoalManager STUCK silent**: goal was at 12.72m when committed; robot closed to 5.32m
   (7.4m progress, well above 0.5m threshold). `committed_dist - cur_dist = 7.4m` permanently
   above STUCK_MIN_PROGRESS. After the robot froze at 5.32m the check sees "already made
   great progress" and never re-arms. (**Partially mitigated by Ch23 fix 1**; a deeper fix
   would reset `committed_dist` after each successful progress interval.)

3. **min_commit_duration=10s over-restraint**: only 7 goal commits total, missing large
   unexplored zones that appeared en route. (**Fixed in Ch23**)

---

## Change 24 — Vanish-guard: hold committed goal when robot is close

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 10 exposed a figure-8 oscillation in the (12.5, −8) area. The robot
alternated between goals (12.86, −5.06) and (12.40, −11.20) every ~10s for 163 seconds with
zero new cells mapped. Root cause: frontier cluster at the committed goal disappears from the
map right as the robot approaches it (the sensor maps those cells as free/occupied), triggering
an immediate vanish-based switch in `_pick_with_hysteresis`. The controller never fires
`GOAL_REACHED` because `_goal_cb` replaces the goal before the 2m radius check. `min_commit_duration`
cannot help because this uses the vanish path, not the score path.

**Fix**: Before doing a vanish-based switch, check whether the robot is already within
`goal_vanish_dist` (3m) of the committed position. If so, the cluster was mapped by the
robot's own sensor during approach — return the committed coordinates unchanged so the
controller can declare `GOAL_REACHED` naturally. Only switch immediately if the robot is
far from the vanished goal (genuine map change, not just self-mapping).

```python
if not near_old:
    cur_dist = np.hypot(robot_xy[0] - self._committed[0],
                        robot_xy[1] - self._committed[1])
    if cur_dist <= self.goal_vanish_dist:
        return float(self._committed[0]), float(self._committed[1])
    self._commit(best.wx, best.wy, robot_xy, now)
    return best.wx, best.wy
```

Safety: if the robot is stuck near the held goal (wall), `_check_and_blacklist_if_stuck`
fires after 15s with <0.5m progress and blacklists it, breaking the hold.

**Observed impact**: 🔲 Not yet tested.

---

## Change 25 — Reduce yaw gains (turning too fast)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot visually turns too fast. With `KP_YAW=0.10`, a 180° goal reversal
produces `yaw_cmd ≈ 0.31`. `ESCAPE_YAW=0.40` (BLOCKED spin) compounds this. Session 10
figure-8 made it especially visible — continuous high-speed reversals.

**Fix**:
- `KP_YAW`: 0.10 → 0.07 — reduces max navigation yaw from 0.31 to 0.22 (a 180° turn)
- `ESCAPE_YAW`: 0.40 → 0.20 — reduces BLOCKED/escape spin rate

New relationship: `ESCAPE_YAW (0.20) ≈ SCAN_YAW * 2.5 ≈ max nav yaw * 0.9`. The escape
spin is now a committed timed maneuver at a moderate rate rather than a high-speed spin.

**Observed impact**: 🔲 Not yet tested.

---

## Session 10 — Findings (2026-05-19)

**Duration**: 283s  
**Files**: `logs/2026-05-19_17-17-59_*.csv`  
**Code state**: Ch20–22 applied; Ch23 applied just before next session.

### Summary

Best coverage session so far: **+16,543 cells** (9,676 → 26,219) in 283s. Phase 1 (0–120s)
was excellent with rapid multi-directional exploration reaching y≈−13. Phase 2 (120–283s)
lost 163s to a figure-8 oscillation between two goals that the robot could never complete.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active exploration | 0–120s | 9,676 → 26,019 | Many goal commits, south zone reached (y≈−13), +16,343 cells |
| 2 — Figure-8 loop | 120–283s | 26,019 → 26,219 | Oscillating between (12.86,−5.06) and (12.40,−11.20), only +200 cells |

### What worked

- Many goal commits in phase 1 (more spatial coverage than sessions 8–9)
- BLOCKED events brief (Ch21 working); no long BLOCKED stalls
- Depth hold stable throughout
- South zone (y≈−12 to −14) explored well again

### Root cause of figure-8 loop

Robot approaches (12.40, −11.20) → at 1.4m (inside `GOAL_RADIUS=2.0m`), the frontier
cluster is mapped by the sensor → `near_old` empty → GoalManager publishes (12.86, −5.06)
at 8m away → robot reverses north → approaches (12.86, −5.06) → cluster vanishes at ~2.1m
→ GoalManager publishes (12.40, −11.20) → repeat. `min_commit_duration` can't prevent
vanish-based switches. **Fixed by Change 24.**

Additionally, 180° goal reversals with `KP_YAW=0.10` produced visually fast spinning.
**Fixed by Change 25.**

---

## Change 26 — A* path planning (replaces reactive BLOCKED machinery)

**Date**: 2026-05-19  
**Files**: `path_planner.py` (new), `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Replace the sensor-reactive BLOCKED state machine with proactive A* path
planning on the OccupancyGrid.  Reactive avoidance had three failure modes that persisted
across sessions 4–10: (1) robot gets trapped in BLOCKED spin loops at complex corners,
(2) figure-8 oscillations caused by approaching frontiers through mapped walls, (3) escape
spins are blind — they often spin into the same obstacle from a different angle.

A* solves all three: it plans around mapped walls before committing to a direction.

**What was removed** (from `waypoint_controller.py`):
- Left/right camera strip processing (`_min_left_dist`, `_min_right_dist`)
- `_obstacle_factor()` surge scaling
- `BLOCKED` state machine and all related state/constants:
  `_blocked_since`, `_escape_yaw_dir`, `_escape_dir_flip_t`,
  `BLOCKED_ESCAPE_TIMEOUT`, `BLOCKED_DIR_MIN_DURATION`,
  `OBSTACLE_SLOW_DIST`, `OBSTACLE_STOP_DIST`
- CSV columns `obs_left_m`, `obs_right_m`, `blocked`

**What was added**:

*`path_planner.py` (pure functions)*:
- `inflate_occupied(grid, radius_cells)` — binary dilation of occupied (100) cells only;
  unknown (−1) cells are never inflated into obstacles (optimistic for exploration)
- `_astar(blocked, start, goal)` — standard A* with 8-connectivity, min-heap, visited set;
  unknown and free cells both traversable; returns list of (row, col) or None
- `plan_path(grid_msg, robot_xy, goal_xy, inflation_m=0.75)` — full pipeline:
  inflate → A* → thin waypoints (every 10th cell); snaps start/goal out of inflated walls
  by finding nearest free cell; returns world-frame `[(x, y), ...]` or `[]` on failure

*`frontier_extractor.py`*:
- `_current_goal_xy` state variable; set in `_publish_goal`
- `_path_pub` on `/frontier_slam/path` (nav_msgs/Path)
- `_replan()` timer at 1 Hz: runs A*, publishes path; warns if A* returns empty

*`waypoint_controller.py`*:
- Subscribes to `/frontier_slam/path`; drives toward `_path[_wp_idx]` instead of raw goal
- Waypoint advance: while closest waypoint is within `WAYPOINT_ADVANCE_DIST=1.5m`, advance index
- On new path arrival: reset `_wp_idx` to the nearest waypoint to current position (handles 1Hz replanning smoothly)
- On new goal (>1m change): clear path and reset index so stale waypoints aren't followed
- **Initial 360° scan**: spin for `INIT_SCAN_DURATION=20.0s` on first odom before navigating — gives A* a populated local map before the first path is planned
- Emergency stop: if `obs_m < EMERGENCY_STOP_DIST=0.4m` → zero surge (single line, no state machine); last resort for dynamic obstacles not in the map
- `CTRL_STUCK` position-stuck detection unchanged (safety net; should rarely fire now)

**CSV schema change** (controller):
```
Before: … obs_m, obs_left_m, obs_right_m, blocked, event
After:  … obs_m, path_len, wp_idx, event
```

**Why inflation_m=0.75?**  
BlueROV2 body radius ≈ 0.3m. 0.75m gives ≈ 0.45m clearance margin beyond the body,
keeping the planned path well away from walls and preventing the robot from brushing
inflated-but-passable gaps it would physically block.

**Why unknown = free?**  
Treating unknown cells as obstacles would prevent planning into unexplored space — the robot
could never reach frontier goals that are surrounded by unmapped area.  Optimistic planning
is standard practice in frontier exploration.

**Observed impact**: 🔲 Not yet tested.

---

## Change 28 — GoalManager vanish-guard: release committed goal when robot has arrived

**Date**: 2026-05-19  
**Files**: `goal_manager.py`, `frontier_extractor.py`

**Objective**: Session 12 showed GOAL_REACHED still looping indefinitely every ~12s, even
after Ch27's fix to `_goal_reached_at`.

**Root cause**: The vanish-guard (Ch24) held the committed position whenever the cluster
disappeared and `cur_dist ≤ goal_vanish_dist=3.0m`. This was correct for the approaching
(2–3m) case. But when the robot was already inside `GOAL_RADIUS=2.0m`, the cluster was
gone, the vanish-guard held (6.13, −5.22), and the extractor kept republishing it. The
controller cleared `_goal=None` after GOAL_REACHED_TIMEOUT=10s, entered SCAN for 1 tick,
then received (6.13, −5.22) again as a "new" goal (was None → something = `goal_changed`),
resetting `_goal_reached_at` and restarting the cycle.

**Fix**: Split the vanish-guard into two sub-cases based on `goal_radius`:
```python
if cur_dist <= self.goal_vanish_dist:
    if cur_dist > self.goal_radius:
        return committed   # still approaching — hold so controller reaches it (Ch24)
    # inside goal_radius: robot has arrived — switch to next frontier immediately
    self._commit(best.wx, best.wy, robot_xy, now)
    return best.wx, best.wy
```
The `goal_radius=2.0` parameter is added to GoalManager and passed from the extractor.
Result: once the robot is within 2.0m of the committed position, GoalManager proactively
switches to the next frontier — the extractor starts publishing a different goal, and the
GOAL_REACHED loop cannot form.

**Why the figure-8 does not return**: The original figure-8 (session 10) involved the
vanish-guard being bypassed at ~1.4m distance (inside goal_radius). The Ch24 fix held the
goal down to 0m. The new fix holds from `goal_vanish_dist` down to `goal_radius` (3.0m →
2.0m), then switches. At 1.4m, the switch fires — but the robot has already mapped the
frontier, so the switched-to goal is a genuinely new candidate, not the previous one in
a loop.

**Observed impact**: 🔲 Not yet tested.

---

## Change 27 — Fix GOAL_REACHED reset on republish + shorten initial scan

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 11 exposed two regressions from the Ch26 rewrite:

1. **GOAL_REACHED stuck forever**: Robot reached goal (7.60, −7.50) at t≈42s and spun in
   GOAL_REACHED for 82s with no exit. Root cause: `_goal_cb` reset `_goal_reached_at = None`
   unconditionally on every call. The extractor republishes the same goal at 0.5 Hz; every
   2s the timer was cleared, preventing the 10s `GOAL_REACHED_TIMEOUT` from accumulating.
   Identical pattern to the Ch23 `_stuck_ref_pos` bug — same extractor republish, same fix.

2. **INIT_SCAN too long (20s)**: Robot completed 3–4 full rotations instead of ~1. Also,
   the GoalManager commits to the first goal at t=0 and its 15s stuck timer fires at t≈17s
   (while the robot is still in INIT_SCAN and stationary), blacklisting the first goal
   before the robot even moves (`STUCK_BLACKLIST` for (12.50, 0.00) visible in session 11
   extractor CSV at t≈17s).

**Fix 1** — move `_goal_reached_at = None` inside `if goal_changed:`:
```python
self._goal = new_goal
if goal_changed:            # ← was outside this block
    self._goal_reached_at = None
    self._path     = []
    ...
```

**Fix 2** — `INIT_SCAN_DURATION`: 20.0s → 5.0s (~1 rotation, finishes before GoalManager
stuck_timeout=15s fires).

**Observed impact**: 🔲 Not yet tested.

---

## Session 11 — Findings (2026-05-19)

**Duration**: ~124s  
**Files**: `logs/2026-05-19_17-54-27_*.csv`  
**Code state**: Ch26 applied (A* path planning, INIT_SCAN, no BLOCKED machinery).

### Summary

No obstacle hits at all — A* routing is working. Navigation clean: robot went from start
to (13.10, −6.90) then to goal (7.60, −7.50) smoothly with correct path following.
Coverage: 10,581 → 22,307 cells (+11,726) in 124s.  Two bugs found and fixed (Ch27).

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — INIT_SCAN | 0–20s | 10,581 → 18,366 | 20s spin — 3–4 rotations (too long) |
| 2 — Navigation | 20–42s | 18,366 → 22,079 | Clean A* path follow, no obstacles hit |
| 3 — GOAL_REACHED loop | 42–124s | 22,079 → 22,307 | Stuck spinning at (7.82,−7.27) — GOAL_REACHED_TIMEOUT never fired |

### Root causes found

1. **GOAL_REACHED never times out**: extractor republish of same goal resets
   `_goal_reached_at` every 2s. 10s timeout cannot accumulate. **Fixed Ch27.**

2. **INIT_SCAN 20s causes GoalManager false STUCK**: first goal (12.50, 0.00) blacklisted
   at t≈17s because robot hadn't moved (still in INIT_SCAN). **Fixed Ch27 (5s scan).**

3. **Extra goal switch at t≈35s**: after reaching area near first goal, GoalManager's
   vanish-switch fired for (13.99,−2.49) within 2s of committing (robot far from it, map
   updated) — causing one extra turn. Not a bug; min_commit_duration correctly did not
   block the vanish path.

---

## Change 29 — Rich debug visualisation + INFLATION_M single source of truth

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`, `path_planner.py`

**Objective**: Add live visualisation so wall crashes and stuck behaviour can be diagnosed
in RViz2 / rqt_image_view without relying solely on CSV post-mortem analysis.
Also eliminate the duplicated `INFLATION_M` constant.

**What was added** (`frontier_extractor.py`):

- `/frontier_slam/inflated_map` (`OccupancyGrid`): same grid as `/projected_map` but with
  inflation zone encoded as value 50 (walls stay 100, free stays 0). Display alongside the
  raw map in RViz2 and toggle independently to see exactly what A* treats as blocked.

- `/frontier_slam/debug_image` (`sensor_msgs/Image`, rgb8): top-down composite image updated
  at 1 Hz alongside `_replan`. Colour key:
  - Dark gray (80,80,80) = unknown
  - Light gray (210,210,210) = free
  - Near-black (20,20,20) = occupied wall
  - **Orange (200,100,50) = inflation zone** — the safety margin A* avoids
  - **Green (0,220,0) = current A\* path waypoints**
  - **Red cross = goal position**
  - **Blue cross = robot position**
  View with `ros2 run rqt_image_view rqt_image_view /frontier_slam/debug_image`

- Goal arrow (`MarkerArray` ns=`goal_arrow`): orange `Marker.ARROW` with tail 2 m behind
  the goal along the robot→goal vector, head at the goal. Shows required approach heading
  at a glance; complements the existing red sphere.

**What was changed** (`path_planner.py`):
- `INFLATION_M = 0.75` promoted to module-level constant (was an inline default argument).
- `plan_path` default argument now references `INFLATION_M` directly.

**What was removed** (`frontier_extractor.py`):
- `INFLATION_M = 0.75` class constant (was a duplicate); now imported from `path_planner`.

**New state** (`frontier_extractor.py`):
- `_current_path: list` — last A\* waypoints, stored in `_replan` before publishing the Path
- `_inflated_grid: np.ndarray | None` — inflated grid recomputed each `_replan` tick

**Observed impact**: 🔲 Not yet tested.

---

## Change 30 — Fix crash when STUCK_BLACKLIST exhausts all candidates

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`

**Objective**: Fix `ValueError: cannot convert float NaN to integer` crash in `_replan`.

**Root cause**: When `STUCK_BLACKLIST` fires and every remaining cluster happens to be within
`goal_vanish_dist=3.0m` of the just-blacklisted goal, `GoalManager.select()` returns
`GoalSelection(nan, nan, 0, 'STUCK_BLACKLIST', ...)`. The extractor's early-return only
checked for `event == 'ALL_BLACKLISTED'`, so the NaN goal fell through to `_publish_goal(nan, nan)`,
which set `_current_goal_xy = [nan, nan]`. On the next 1Hz `_replan` tick,
`_current_goal_xy is not None` passed (NaN array is not None), and `plan_path` crashed on
`int(float('nan'))`.

**Fix 1** — merge the two "no valid goal" cases in `_update`:
```python
if selection.event == 'ALL_BLACKLISTED' or math.isnan(selection.gx):
    self._current_goal_xy = None
    self._current_path    = []
    ...
    return
```
Clearing `_current_goal_xy` prevents `_replan` from planning to a stale position while
waiting for the blacklist to expire.

**Fix 2** — defensive NaN guard in `_replan`:
```python
if self._current_goal_xy is not None and not np.any(np.isnan(self._current_goal_xy)):
    waypoints = plan_path(...)
```

**Observed impact**: 🔲 Not yet tested.

---

## Change 31 — Debug image: heading arrow, path lines, state colour bar

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`

**Objective**: Make the debug image actually useful for diagnosing stuck behaviour and
wall crashes.

**Robot marker** (was: blue cross):
- Now: **arrow** whose direction matches the robot's heading (yaw from odometry quaternion)
  and whose length scales with ground speed (`arm = 4 + speed_m_s * 20 px`).
  At `MAX_SURGE=0.4 m/s` the arrow is ~12 px; stationary → 4 px minimum.
- Colour encodes extractor state:
  - Blue  = navigating normally
  - Yellow = stuck_pct ≥ 50 % (approaching timeout)
  - Orange = stuck_pct = 100 % (maxed, blacklist imminent)
  - Cyan  = no valid goal (scanning / all blacklisted)

**Path** (was: isolated 3×3 dots per waypoint):
- Now: connected **line segments** between consecutive waypoints via `_draw_line`
  (numpy linspace interpolation, no OpenCV required).

**Status bar**: 4-pixel coloured strip at the top of the image, same colour key as the
robot arrow.

**New helpers added**:
- `_draw_line(img, r0, c0, r1, c1, color)` — static, numpy-only line drawing
- `_robot_arrow_color()` — maps stuck_pct / goal state to a display colour

**New `_odom_cb` fields**:
- `_robot_yaw` — extracted via `yaw_from_quat` (imported from `control_utils`)
- `_robot_speed` — `hypot(twist.linear.x, twist.linear.y)` (body-frame horizontal speed)

**Coordinate note**: in the pre-flip OccupancyGrid image, col↔X(North) and row↔Y(East).
Arrow direction `(dc, dr) = (cos(yaw), sin(yaw))` maps correctly after `np.flipud`:
North = right, East = up in the displayed image.

**Observed impact**: 🔲 Not yet tested.

---

## Session 13 — Findings (2026-05-20)

**Duration**: ~95s  
**Files**: `logs/2026-05-20_13-41-51_*.csv`  
**Code state**: Ch26–31 applied (A* planning, debug viz, NaN crash fix).

### Summary

Session crashed into a wall immediately after INIT_SCAN, then recovered partially before freezing permanently at (7.07, 3.20) with obs=0.20m for the remaining ~40s.

### Phase breakdown

| Phase | t | Robot pos | obs_m | path_len | Behaviour |
|---|---|---|---|---|---|
| INIT_SCAN | 0–10s | (0,0) | 4–6m | 0 | Spinning, mapping |
| Direct-goal crash | 10–20s | →(5.75,0.35) | →0.26 | **0** | A* failed → MAX_SURGE straight into wall |
| EMERG_STOP freeze | 20–36s | (~5.9,−0.3) | 0.20–0.29 | 0 | Frozen at wall, GoalManager cycling |
| A* backward path | 36–46s | →(0.5,−0.2) | clears | 7 | Routes back to (−0.18,0.12) ✓ |
| Second direct-goal | 46–60s | →(1.9,6.9) | →0.21 | 0 | A* failed again → another wall hit |
| A* path + reach | 60–73s | →(6.3,3.4) | clears | 4 | Clean path following ✓ |
| Final permanent freeze | 73–end | (7.07,3.20) | **0.20** | 0 | EMERG_STOP from tick 1, no escape |

### Root causes

1. **A* consistently failing on initial goals**: `path_len=0` whenever goal is in the direction
   of a known wall. A* returns no path — robot falls back to direct heading at MAX_SURGE and
   crashes. Diagnosis requires seeing the debug image inflated map. Likely cause: inflation
   radius blocks the only corridor, or goal cell is inside inflated zone.

2. **No speed reduction near obstacles**: binary hard-stop at 0.4m; robot arrives at 0.40 m/s.
   **Fixed in Ch32.**

3. **CTRL_STUCK blind to EMERG_STOP freeze**: when EMERG_STOP keeps surge=0, the stuck
   detector never arms (`surge < STUCK_SURGE_MIN=0.15`). Robot can be frozen indefinitely.
   At t=73s, obs=0.20m from the first tick of a new goal — robot never moved before freezing.

---

## Change 32 — Progressive obstacle slow-down + reduce MAX_SURGE

**Date**: 2026-05-20  
**Files**: `waypoint_controller.py`

**Objective**: Session 13 showed the robot arriving at walls at full speed (0.40 m/s) with
only a binary hard-stop at 0.4m as protection. The EMERG_STOP freeze is invisible to
CTRL_STUCK, leaving the robot permanently stuck.

**Fix — surge ramp**:
Replace the single-threshold hard-stop with a continuous linear ramp:
```
obs ≥ OBS_SLOW_DIST (2.0m) → surge unchanged (full speed)
obs in [0.4m, 2.0m]        → surge *= (obs − 0.4) / (2.0 − 0.4)
obs < EMERGENCY_STOP_DIST (0.4m) → surge = 0, event = 'EMERG_STOP'
```
At 1.2m the robot is moving at half speed; at 0.8m, quarter speed. This gives the
controller time to steer away before actually stopping.

**Fix — lower MAX_SURGE**:
`MAX_SURGE`: 0.40 → 0.25 m/s. Gives A* replanning (1Hz) enough time to react to
newly-mapped obstacles before the robot reaches them at full speed.

**Observed impact**: 🔲 Not yet tested.

---

## Session 14 — Findings (2026-05-20)

**Duration**: ~70s  
**Files**: `logs/2026-05-20_13-47-57_*.csv`  
**Code state**: Ch32 applied (progressive ramp, MAX_SURGE=0.25).

### Summary

Ramp working: robot slowed from 0.25→0 over the last 1.6m before the wall (obs 2.0→0.37m).
No hard crash. But frozen indefinitely once at wall — same root cause as session 13.

### Key findings

- **path_len=0 the entire session**: A* fails even from open water (robot at (0,0), goal at
  (9.0,0.0)). Robot falls back to direct goal heading and reaches the wall at x≈5.75m.
- **EMERG_STOP correct but no escape**: backs nothing, freezes at obs≈0.25m.
- **GoalManager STUCK silent again**: robot closed 8.60m→3.26m = 5.34m total progress.
  `committed_dist - cur_dist` permanently above `stuck_min_progress=0.5m` so the check
  reads "not stuck" even after the robot has been frozen for 40+ seconds. **Fixed in Ch33.**
- **CTRL_STUCK blind**: surge=0 from EMERG_STOP → stuck detector never arms.

---

## Change 33 — Back-surge escape + GoalManager sliding-window stuck detection

**Date**: 2026-05-20  
**Files**: `waypoint_controller.py`, `goal_manager.py`

**Objective**: Two independent failure modes both prevent recovery from a wall encounter:
the controller freezes in place and the GoalManager never detects it as stuck.

**Fix 1 — back-surge** (`waypoint_controller.py`):
When `obs < EMERGENCY_STOP_DIST (0.4m)`, apply `surge = -BACK_SURGE_SPEED (0.12 m/s)`
instead of zeroing surge. The robot actively reverses away from the wall, moving back into
the free zone where A* can replan from a valid starting cell.

Sequence: approach wall → ramp reduces surge [2.0m→0.4m] → back-surge kicks in [<0.4m]
→ robot backs up → obs rises → ramp re-engages → A* has a clear start position and replans
→ path found → follows route around wall.

**Fix 2 — sliding-window stuck detection** (`goal_manager.py`):
Replace the one-shot `committed_dist - cur_dist` check with a progress clock:
- Track `_closest_dist` (minimum distance to goal ever seen) and `_closest_t` (when it was set).
- Clock resets whenever the robot closes the gap by > `stuck_min_progress (0.5m)`.
- STUCK fires when `now - _closest_t >= stuck_timeout (15s)`.

Before: "robot made 5m total progress → permanently not stuck."
After: "robot hasn't gotten any closer in 15s → stuck."

`stuck_pct` now shows % of stuck_timeout elapsed since last progress (not since commitment),
which is the genuinely meaningful quantity.

**Observed impact**: ✅ Session 14 (2026-05-20): back-surge prevented the wall-freeze completely. Robot still ends up stuck inside the 0.75m soft inflation zone where A* cannot find a path. Fixed by Ch34.

---

## Change 34 — Dual-inflation A* (soft zone + hard wall)

**Date**: 2026-05-20  
**Files**: `path_planner.py`, `frontier_extractor.py`

**Problem**: When a frontier goal lands inside the 0.75m inflation zone, A* treats every
cell there as hard-blocked, so `nearest_free` snaps the goal to the inflation boundary.
If the corridor is also sealed by inflation, no path is ever found, and the robot freezes.

**Fix — dual inflation layers**:
- `HARD_INFLATION_M = 0.20 m` — true hard wall for A*. Only cells within 0.20m of an
  obstacle are `inf` in the cost grid. Narrow corridors remain passable.
- `INFLATION_M = 0.75 m` (soft) — cells between 0.20m and 0.75m are penalised by
  `SOFT_COST = 8.0`, so A* naturally routes around them, but can pass through when there
  is no alternative (e.g. the goal is inside the soft zone).
- `nearest_free` now snaps only against the hard boundary — a goal sitting in the soft
  zone is valid and reachable.

Changes in `path_planner.py`:
- Added `HARD_INFLATION_M = 0.20` and `SOFT_COST = 8.0` constants.
- `_astar` signature changed from `blocked: np.ndarray` (bool) to `cost_grid: np.ndarray`
  (float). `inf` = wall, `SOFT_COST` = avoidance zone, `1.0` = free. Step cost is
  `(1.414 if diagonal else 1.0) * cell_cost`.
- `plan_path` renamed parameter `inflation_m` → `soft_inflation_m`, added `hard_inflation_m`.
  Builds `cost_grid` from two `inflate_occupied` calls.

Changes in `frontier_extractor.py`:
- Imports `HARD_INFLATION_M` from `path_planner`.
- Added `_hard_inflated_grid` state, computed alongside `_inflated_grid` in `_replan`.
- Debug image: hard zone shown as dark red `(150, 30, 30)`, soft zone stays orange `(200, 100, 50)`.
- Inflated map: hard zone → 75, soft zone → 50.

**Observed impact**: ✅ Session 15 (2026-05-20): A* finds paths and robot reaches multiple goals. Two remaining issues fixed by Ch35: (1) back-and-forth oscillation between visited goals, (2) surge ramp engaging too early.

---

## Change 35 — Arrival blacklist + tighter obstacle slow-down

**Date**: 2026-05-20  
**Files**: `goal_manager.py`, `waypoint_controller.py`

**Problem 1 — back-and-forth oscillation** (session 15): After reaching a frontier, GoalManager
immediately commits to the next best candidate, which is often the other side of the same pillar.
The robot then reaches that goal and commits back. Neither goal was blacklisted because only
STUCK goals went on the blacklist.

**Fix**: Add `arrival_blacklist_duration = 20 s`. When the robot arrives (cur_dist ≤ goal_radius)
in `_pick_with_hysteresis`, append the committed goal to the blacklist with a 20s expiry before
committing to the next one. This prevents immediately re-picking a recently visited frontier.
Parameter sits alongside `blacklist_duration` (60 s, used for STUCK goals).

**Problem 2 — surge ramp starting too early**: `OBS_SLOW_DIST = 2.0 m` caused surge to begin
ramping down during normal circumnavigation (obs ~1.4–1.8 m while routing around the pillar).
The A* soft zone already steers the path away from walls; the controller ramp is last-resort
safety, not primary path shaping.

**Fix**: `OBS_SLOW_DIST` 2.0 m → 1.5 m. Ramp now only engages when genuinely close to an
obstacle, not during planned detours through tight corridors.

**Observed impact**: ✅ Session 16 (2026-05-20): arrival blacklist confirmed working (no back-and-forth). But STUCK fires 4× in 2.5 min because A* routes through soft zone → surge drops to 0.06 m/s → underwater drift → robot moves sideways rather than toward goal → GoalManager fires after 15s. Fixed by Ch37.

---

## Change 36 — Grid padding so A* can route near map boundaries

**Date**: 2026-05-20  
**Files**: `path_planner.py`

**Problem**: When a wall or obstacle touches the edge of the OccupancyGrid, A* cannot route
around it because no cells exist outside the grid bounds. The robot gets stuck with no path.

**Fix**: In `plan_path`, pad the raw grid with `PAD_CELLS = 5` layers of unknown cells (`-1`)
on all four sides before planning. Unknown cells have cost 1.0 (same as free space), so A*
can route through the padded border to go around boundary obstacles. The origin `ox/oy` is
shifted by `PAD_CELLS × resolution` to keep world coordinates consistent. The padding only
affects planning — visualization still uses the original unpadded map.

**Observed impact**: 🔲 Not yet tested.

---

## Change 37 — Three-zone A* cost grid + longer stuck timeout

**Date**: 2026-05-20  
**Files**: `path_planner.py`, `frontier_extractor.py`

**Problem** (session 16): A* routed paths through the soft zone (0.75m from walls). Underwater
drift has no friction correction, so any deviation put the robot inside the soft zone where surge
dropped to 0.06–0.12 m/s. The robot moved sideways rather than toward the goal, distance
increased, and GoalManager fired STUCK four times in 2.5 min — goal abandoned before any area
was properly explored.

**Fix 1 — Planning zone** (`path_planner.py`):
Add a third zone between the soft zone and free space:
- Free space (d > 1.5m): cost = 1.0 — A*'s first preference
- Planning zone (0.75m < d ≤ 1.5m): cost = PLAN_COST = 3.0 — moderate penalty
- Soft zone (0.20m < d ≤ 0.75m): cost = SOFT_COST = 8.0 — only if forced
- Hard zone (d ≤ 0.20m): cost = inf — never

A* now routes paths through corridors ≥1.5m from walls when available. The planning zone acts
as a buffer between the desired path and the sensor slow-down zone — even with drift, the robot
stays out of the orange soft zone on most paths.

**Fix 2 — Longer stuck timeout** (`frontier_extractor.py`):
`stuck_timeout` 15s → 30s. When a valid A* path arcs around an obstacle, the robot's distance
to the final goal temporarily increases. 15s was too short to tolerate this; 30s gives the robot
time to clear an obstacle arc before declaring it stuck.

Visualisation updated: planning zone shown as olive/khaki `(180, 160, 60)` in debug image,
value 35 in the inflated OccupancyGrid.

**Observed impact**: ✅ Session 17 (2026-05-20): three-zone planning confirmed wider paths. But rapid silent goal switches (score-based) still firing — root cause identified in Ch38.

---

## Change 38 — Remove score-based goal switching: commit-until-done model

**Date**: 2026-05-20  
**Files**: `goal_manager.py`, `frontier_extractor.py`

**Problem** (session 17): A cluster appeared 3.5m from the robot while it was heading toward a
10m goal. The silent score-based switch fired because `3.5 < 8.25 − 3.0 = 5.25m` — correct
by the old logic but wrong behaviour. A robot that has already traveled 2m toward a goal should
not abandon it because a nearby cluster appeared. The current-distance comparison made the robot
progressively *less* committed the more progress it made.

**Fix**: Remove score-based switching entirely.  A committed goal is held until:
  1. **Arrived** — robot within `goal_radius`, cluster has vanished.
     Brief arrival blacklist (20s) prevents immediate re-pick.
  2. **Stuck** — no progress toward goal for `stuck_timeout` (30s).
     Stuck blacklist (30s) prevents immediate re-pick.

`_pick_with_hysteresis` → renamed `_pick_committed`. Logic:
- Cluster still in frontier list → track map-drift of committed position.
- Cluster vanished + robot arrived → arrival blacklist + pick best.
- Cluster vanished + robot not arrived → keep heading to last known position
  (STUCK will fire if progress genuinely stalls).
- No score comparison, no hysteresis, no min_commit_duration.

Removed parameters: `switch_hysteresis`, `min_commit_duration` (dead code).

**Observed impact**: ⚠️ Session 18 (`2026-05-20_15-16-03`): goal commitment held correctly — no silent score-based switches. Robot committed to (10.40, 0.00) and stayed on it throughout. However, `STUCK_BLACKLIST` fired at t≈90s despite the robot actively following a valid A* path at full surge (0.25 m/s). Root cause: STUCK measures straight-line distance to goal; A* routed a large southward arc around a structure, temporarily increasing goal-distance from 8.19m to ~9.6m. The stuck timer ran 30s with no improvement in goal-distance → false positive. Fixed by Ch39.

---

## Change 39 — Fix false STUCK during A* detour arcs (displacement-based reset)

**Date**: 2026-05-20  
**Files**: `goal_manager.py`

**Problem** (session 18): STUCK_BLACKLIST fired while the robot was correctly following an A* path. The robot committed to goal (10.40, 0.00), made its closest straight-line approach of 8.19m, then A* routed it south along a 5-6m arc to go around a structure. During that arc goal-distance increased from 8.19m to ~9.6m over ~28s. The stuck clock (`now - _closest_t`) accumulated 30s without a reset because the stuck detector only resets on `cur_dist < _closest_dist - 0.5m`. The robot was never physically stuck — it was moving at full surge (0.25 m/s) with obs=2-4m and a valid path (path_len=22-33).

**Root cause**: STUCK was measuring progress toward the final goal (straight-line), not progress along the planned path. For any significant A* detour, those diverge — the robot gets further from the goal while actively navigating around an obstacle.

**Fix**: Add a displacement-based reset to the stuck sliding window. In addition to resetting when `cur_dist` improves by `stuck_min_progress (0.5m)`, also reset when the robot has physically moved `stuck_min_progress` from its position at the last reset. This catches detour arcs (robot moves several meters → constant resets) without false-passing a genuinely frozen robot (moves <0.3m from drift → no reset).

New state: `_closest_ref_pos` — robot position at the time the stuck reference was last set.

```python
# Goal-distance progress (existing):
if cur_dist < self._closest_dist - self.stuck_min_progress:
    self._closest_dist    = cur_dist
    self._closest_t       = now
    self._closest_ref_pos = robot_xy.copy()
# Displacement progress (new — catches A* arcs):
elif (self._closest_ref_pos is not None and
      np.hypot(robot_xy[0] - self._closest_ref_pos[0],
               robot_xy[1] - self._closest_ref_pos[1])
      >= self.stuck_min_progress):
    self._closest_t       = now
    self._closest_ref_pos = robot_xy.copy()
```

`_closest_dist` is not updated on displacement reset — it continues to track the best
goal approach ever, so a robot that drifts sideways without approaching doesn't get credit.

**Session 18 replay**: at t≈1779286591 (closest_dist=8.19m, ref_pos=(3.20,-3.89)), the robot
moved to (3.68,-4.70) — displacement 0.93m > 0.5m → timer resets. Then to (4.02,-5.55) —
another 0.72m → resets again. The arc covers ~6m total, resetting the timer ~12 times.
STUCK would never fire during active navigation.

**Observed impact**: 🔲 Not yet tested.

---

## Current parameter snapshot

### frontier_extractor.py / goal_manager.py
| Parameter | Value | Purpose |
|---|---|---|
| `MIN_CLUSTER_CELLS` | 5 | Discard tiny frontier clusters (noise) |
| `MIN_EXPLORE_DIST` | 3.0 m | Skip frontiers the robot is already at |
| `GOAL_VANISH_DIST` | 3.0 m | Committed goal "gone" if no cluster within this radius |
| `GOAL_RADIUS` | 2.0 m | Robot "arrived" threshold — GoalManager switches when cur_dist ≤ this (Ch28) |
| `STUCK_TIMEOUT` | 30.0 s | Abandon goal after this long with no progress (Ch37, was 15s) |
| `STUCK_MIN_PROGRESS` | 0.5 m | Minimum distance closed to not be "stuck" |
| `BLACKLIST_DURATION` | 30.0 s | Blacklist duration after STUCK — goal abandoned |
| `ARRIVAL_BLACKLIST_DURATION` | 20.0 s | Blacklist duration after goal reached — prevents re-picking (Ch35) |
| `UPDATE_HZ` | 0.5 Hz | Frontier re-evaluation rate |
| `REPLAN_HZ` | 1.0 Hz | A* path replanning rate |

### waypoint_controller.py
| Parameter | Value | Purpose |
|---|---|---|
| `KP_YAW` | 0.07 | Heading P-gain |
| `KP_SURGE` | 0.25 | Forward speed P-gain |
| `KP_HEAVE` | 0.40 | Depth-hold P-gain |
| `MAX_SURGE` | 0.25 | Surge clamp (Ch32, was 0.40) |
| `GOAL_RADIUS` | 2.0 m | "Goal reached" threshold |
| `GOAL_REACHED_TIMEOUT` | 10.0 s | Clear stale goal and enter scan if no new goal arrives |
| `SCAN_YAW` | 0.08 | Rotation speed during scan / initial scan |
| `INIT_SCAN_DURATION` | 5.0 s | Startup spin duration before navigating (Ch27, was 20s) |
| `WAYPOINT_ADVANCE_DIST` | 1.5 m | Advance to next path waypoint when this close (Ch26) |
| `OBS_SLOW_DIST` | 1.5 m | Begin linear surge ramp-down at this distance (Ch35, was 2.0m) |
| `EMERGENCY_STOP_DIST` | 0.4 m | Below this: switch from ramp to back-surge (Ch33) |
| `BACK_SURGE_SPEED` | 0.12 m/s | Backward speed when obstacle inside EMERGENCY_STOP_DIST (Ch33) |
| `ESCAPE_YAW` | 0.20 | Spin rate for CTRL_STUCK escape |
| `ESCAPE_DURATION` | 4.0 s | How long to spin after CTRL_STUCK trigger |
| `STUCK_SURGE_MIN` | 0.15 | Min surge to activate position-stuck detection |
| `STUCK_WINDOW` | 5.0 s | Position-stuck detection window |
| `STUCK_MOVE_MIN` | 0.25 m | Min movement expected in STUCK_WINDOW |
| `CTRL_HZ` | 10.0 Hz | Control loop rate |

### path_planner.py
| Parameter | Value | Purpose |
|---|---|---|
| `HARD_INFLATION_M` | 0.20 m | Hard A* wall — never traversed (Ch34) |
| `INFLATION_M` | 0.75 m | Soft zone — last-resort passage, cost×8 (Ch34) |
| `PLAN_INFLATION_M` | 1.50 m | Planning margin — A* prefers staying outside, cost×3 (Ch37) |
| `SOFT_COST` | 8.0 | Cost multiplier inside soft zone (Ch34) |
| `PLAN_COST` | 3.0 | Cost multiplier inside planning zone (Ch37) |
| `PAD_CELLS` | 5 cells | Unknown-cell border added around the map before A* (Ch36) |
| `stride` | 10 cells | Waypoint thinning — keep every 10th path cell |

---

## Log sessions

Raw logs go in `logs/`. From Change 11 onward, CSV files are generated automatically.

| File | Date | Summary |
|---|---|---|
| `2026-05-19_13-46-59_session1_raw.log` | 2026-05-19 | Pre-CSV manual log. 3 stuck detections. Loop established between (14.7,2.2) and (10,9). Z drift 8.5→13.8m. |
| `2026-05-19_14-51-54_*.csv` | 2026-05-19 | First CSV session. Depth hold confirmed working (z drift 0.13m over 82s). Frontiers depleted after 12s, robot froze at GOAL_REACHED for 63s. |
| `2026-05-19_15-05-00_*.csv` | 2026-05-19 | Goal timeout (Ch15) fired correctly. Coverage metrics (Ch16) working. Robot reached (14.44,−4.06), SCAN mode entered but SCAN_YAW=0.008 too slow to reorient. 9756 mapped cells. |
| `2026-05-19_15-22-50_*.csv` | 2026-05-19 | Session 4. BLOCKED at (6.04,−0.05) with obs=0.20m from t=+6s to end (80+ seconds frozen). yaw_cmd=0.00 throughout — aligned BLOCKED with no escape. |
| `2026-05-19_15-40-14_*.csv` | 2026-05-19 | Session 5. BLOCKED at (5.70, 0.56) obs=0.25m for 60+ seconds, then BLOCKED again at (13.88,−0.31). Same aligned-BLOCKED pattern; change 18 not yet applied. |
| `2026-05-19_15-43-04_*.csv` | 2026-05-19 | Session 6. Two failure modes: (1) brief BLOCKED at (5.7,0) cleared by goal change, (2) physical stuck at (9.88,−1.71) for 35+ seconds with obs=2.4m — off-axis wall not in camera FOV. Root cause of Ch18. |
| `2026-05-19_15-55-06_*.csv` | 2026-05-19 | Session 7. 366s. Ch18 confirmed: all BLOCKED events clear in 1–2 ticks. 27,151 mapped cells (2.2× session 6). 89% of coverage in first 55s. Final 231s stuck in y≈9.9 loop — nearest-first oscillation. Slow depth drift (err 0→0.55m). |
| `2026-05-19_16-15-40_*.csv` | 2026-05-19 | Session 8. 311s. Ch19 (frontier scoring) confirmed: robot went south to y=−14, reached installation area. 28,139 mapped cells, 16,100 new (+58% cells/min vs session 7, −43% odometer). 3 STUCK_BLACKLIST events. BLOCKED 32.4% of session (south zone obstacle-dense). Phase 4 (t>214s) stalled: only 4 clusters remain, all unreachable. |
| `2026-05-19_16-50-54_*.csv` | 2026-05-19 | Session 9. 182s. Ch21+22 applied. CTRL_STUCK timer bug exposed: robot frozen 62s at (7.6,−4.1) with no escape trigger. 7 goal commits only (min_commit=10s too conservative). 23,127 mapped cells. Fixed by Ch23. |
| `2026-05-19_17-17-59_*.csv` | 2026-05-19 | Session 10. 283s. Best coverage: +16,543 cells. Excellent phase 1 (0–120s). Phase 2: 163s figure-8 loop between (12.86,−5.06) and (12.40,−11.20) — frontier vanish-switch triggered as robot arrives, controller never fires GOAL_REACHED. Fixed by Ch24. Yaw too fast: Ch25. |
| `2026-05-19_17-54-27_*.csv` | 2026-05-19 | Session 11. Ch26 (A* path planning). No obstacle hits. Clean nav 0→(13.10,−6.90)→(7.60,−7.50). GOAL_REACHED frozen 82s — extractor republish reset `_goal_reached_at` every 2s. Fixed by Ch27. |
| `2026-05-20_13-41-51_*.csv` | 2026-05-20 | Session 13. Wall crash immediately after INIT_SCAN (path_len=0, A* fail → direct heading). EMERG_STOP freeze at obs=0.20m with no escape. Fixed by Ch32 (ramp) and Ch33 (back-surge). |
| `2026-05-20_13-47-57_*.csv` | 2026-05-20 | Session 14. Ramp working (no hard crash). But still frozen at wall with path_len=0 all session — A* not failing gracefully. Back-surge (Ch33) confirmed working. |
| `2026-05-20_14-10-52_*.csv` | 2026-05-20 | Session 15. Ch33+34 applied. Back-surge working. Goal oscillation: robot arrived at frontier, A* planned back to origin, arrival at origin relaunched back to original goal → loop. Added arrival blacklist (Ch35). |
| `2026-05-20_14-32-18_*.csv` | 2026-05-20 | Session 16. STUCK_BLACKLIST fired 4× in ~2.5min. A* routing paths through soft zone → surge ramp → drift → goal-distance increase → stuck. Added 3rd planning zone (Ch37) + STUCK_TIMEOUT 15→30s. |
| `2026-05-20_15-01-26_*.csv` | 2026-05-20 | Session 17. Rapid silent goal switch from (10.10,0.00) to (0.00,−0.36) to (2.25,8.15) within 30s. Score-based hysteresis triggered on committed-dist (shrinks as robot progresses). Removed score switching entirely (Ch38). |
| `2026-05-20_15-16-03_*.csv` | 2026-05-20 | Session 18. Ch38 confirmed: goal held throughout. STUCK_BLACKLIST false positive at t≈90s — A* routed a southward arc, goal-distance increased 8.19→9.6m over 28s → stuck timer expired. Robot was moving at full surge with valid path. Fixed by Ch39 (displacement reset). |

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
**→ Fixed by Changes 12 + 14. Session 2 confirmed: drift reduced to 0.13m over 82s.**

---

## Session 2 — Findings (2026-05-19)

**Duration**: 82s  
**Files**: `logs/2026-05-19_14-51-54_*.csv`

### What worked
- **Depth hold** ✅: z stayed in [8.12, 8.25]m (±0.07m around setpoint 8.14m). Changes 12+14 confirmed.
- **Navigation** ✅: Robot reached (13.74, -3.58) from origin cleanly in ~18s, no collisions.
- **Obstacle avoidance** ✅: Brief 2s BLOCKED at wall, then cleared without collision.

### Issues observed

**1. Exploration terminated after 12 seconds (critical)**  
Frontier count collapsed 7→4→3→3→1→0 as the robot moved forward. The last cluster
(13.98,-4.20) disappeared once the robot navigated past the obstacle and the map updated.
After t=726 the extractor found zero clusters and returned early on every tick, publishing
no new goals. The controller reached the stale goal at t=734 and entered GOAL_REACHED.
The robot then hovered frozen for 63 seconds with no recovery.

Root cause: when frontiers run dry, the controller holds the last received goal forever.
GOAL_REACHED state spins at `SCAN_YAW=0.008` but never clears `_goal`, so it never
enters true SCAN mode. The extractor, finding no clusters, stays silent.

**→ Fixed by Change 15 (goal timeout).**

**2. Why did frontiers run dry so fast?**  
Clusters shrank 7→1 in 11s. Likely the 2D OctoMap only surfaces frontiers within
sensor range; as the robot mapped the near field, small residual patches dropped below
`MIN_CLUSTER_CELLS=5` and disappeared. In open water (obs=inf from t=747) there is
genuinely no structure ahead to generate new unknown cells.
The robot needs to rotate and move to discover new frontiers — which Change 15 enables.
If scanning also fails to find frontiers, `MIN_CLUSTER_CELLS` may need lowering.

---

## Session 7 — Findings (2026-05-19)

**Duration**: 366s (~6 minutes)  
**Files**: `logs/2026-05-19_15-55-06_*.csv`

### What worked

- **Obstacle escape (Ch18)** ✅: Every BLOCKED event resolved in 1–2 ticks. Sessions 4–5
  had BLOCKED lasting 80+ seconds. Here the worst case was 2 consecutive ticks (2s), then
  `obs` jumped from 0.30m to 1.95m. Zero time wasted frozen against a pillar.
- **Coverage** ✅: 27,151 `mapped_cells` — 2.2× the previous session (12,412). Robot
  explored corridors above (y≈9–10) and south (y≈−7) that had never been reached before.
- **Navigation** ✅: Robot moved cleanly at max surge across the full environment with no
  collisions. Heading error typically < 5° during straight runs.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active exploration | 0–55s | 15,348 → 24,157 | Fast goal-switching across 4 corridors; 89% of total coverage |
| 2 — S/N oscillation | 55–90s | 24,157 → 24,739 | Loop between south goal (y≈−7) and north goal (y≈6.8) |
| 3 — New territory | 90–135s | 24,739 → 27,077 | Robot reaches y≈9–10 corridor for first time |
| 4 — Tight corridor loop | 135–366s | 27,077 → 27,151 | 231s in 5m strip, only 74 new cells |

### Issues observed

**1. Back-and-forth loop — nearest-first selection oscillates (critical)**  
In phase 4 the robot locks between goals A=(9.90,8.80) and B=(−2.00 to −3.40,9.80),
±100° heading changes every ~12 seconds. Nearest-first selection picks them alternately
because they are equidistant from the robot's midpoint. The GoalManager's
`SWITCH_HYSTERESIS=3.0m` does not help: it only applies while actively pursuing a goal,
but `GOAL_REACHED` fires and clears `_goal` before the 10s timeout, so each new dispatch
starts fresh. The extractor at 0.5 Hz republishes before the timeout can fire.  
`CTRL_STUCK` does not help either: the robot IS moving 0.25m+ every 5s — it physically
covers the corridor, just not new territory.

Root cause: nearest-first scoring has no preference for information gain. Two equidistant
clusters of very different sizes receive equal priority.

**Potential fix**: Score frontiers by `cluster_size / distance`. A cluster with 20 cells
at 8m beats a cluster with 3 cells at 7m. This breaks equidistance ties toward the more
informative direction and naturally deprioritises tiny residual boundary patches.
See FutureWork §1.

**2. Slow depth drift**  
`depth_err` grows from 0.01m at start to 0.55m by t=135s, then stays there.
Heave output reaches −0.20 (full upward thrust at 0.5m error) but cannot fully
counteract the drift. Likely cause: pitch coupling during aggressive surge — forward
thrust creates a slight nose-down moment, pushing the robot deeper. The setpoint is
locked correctly; the controller just can't overcome the physical effect.
Not critical at current scale but will worsen in longer sessions or deeper environments.

**3. Yaw speed during goal switches**  
On each goal switch (±100°), `yaw_cmd ≈ ±0.17–0.18`. This is fast but below the ESCAPE_YAW
cap. The user's "turns too fast" observation likely refers to ESCAPE_YAW=0.40 firing when
BLOCKED — which is correct behaviour and should stay. Navigation yaw at KP_YAW=0.10
produces smooth turns for small errors but snappy large-angle corrections.

### Key insight: when does exploration actually happen?

89% of session coverage (8,809 cells) was gained in the first 55 seconds. The rest of
the 5-minute session contributed only 2,994 cells. The bottleneck is not obstacle avoidance
or navigation — it is goal selection. Nearest-first with 23 equidistant clusters generates
a random walk among visited frontiers rather than directing the robot toward large unknown
regions. Frontier scoring is the next necessary change.
