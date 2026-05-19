# frontier_slam — Future Work

## 1. Frontier scoring (info-gain / distance)

Currently the extractor always selects the **nearest** frontier centroid.
A better metric is:

```
score = cluster_size / distance_to_robot
```

Large distant clusters beat small nearby ones, steering the robot toward areas
that will yield the most new map coverage per metre travelled. This alone would
make exploration significantly more efficient and reduce revisits.

---

## 2. Goal blacklisting

After a goal is reached, add it to a timed blacklist (e.g. 30 s).
If the same frontier reappears before the timer expires (the robot didn't fully
explore the area on the first pass), it will not be re-selected immediately.
This breaks loops where the robot returns to the same spot repeatedly.

---

## 3. Omnidirectional obstacle avoidance (repulsive field)

The current implementation reads only the forward depth camera for proximity.
With the `simple_rov_maxed` ring of 4 cameras, a lightweight repulsive potential
field can be built at every control step:

1. For each camera (front, right, back, left), compute the minimum range in the
   central FOV strip.
2. Convert to a repulsion vector pointing away from the camera in body frame,
   magnitude ∝ `1 / min_range`.
3. Sum all four vectors into a net repulsion, rotate into world frame, and
   subtract from the desired velocity before mixing into thruster commands.

This gives reactive avoidance in all horizontal directions with no path planner.

---

## 4. A\* path planning on the projected occupancy grid

The P-controller drives in a straight line toward each frontier with no global
awareness of obstacles. For inspection around a structure, even a simple 2D A\*
on the `/projected_map` OccupancyGrid would eliminate wall collisions entirely:

1. On each new goal, run A\* from robot XY to goal XY on the 2-D grid.
2. Publish the path as a sequence of waypoints.
3. The controller tracks the nearest waypoint ahead on the path instead of the
   raw goal.

This is the highest-impact single improvement available and would make the robot
reliably navigate around the offshore station without getting stuck.

---

## 5. 3-D frontier detection

The current extractor works on the 2-D projected OctoMap (`/projected_map`).
This collapses the Z dimension: the robot only seeks frontiers in the XY plane
and maintains its current depth.

True 3-D frontier detection would subscribe to `/octomap_binary`, deserialize
with the Python `octomap` bindings, and find free voxels adjacent to unknown
voxels in all 6 directions. This enables:
- Vertical exploration (above/below the structure)
- Detecting overhangs and cavities invisible to the 2-D projection

Dependency: `python3-octomap` (or equivalent bindings for ROS 2 Jazzy).

---

## 6. Saliency-driven revisits (Suresh et al. 2020)

The long-term goal of this project: replace the greedy nearest-frontier selector
with the dual-behaviour policy from *Active SLAM using 3D Submap Saliency for
Underwater Volumetric Exploration* (ICRA 2020):

- Build a submap-based pose graph alongside the OctoMap.
- Score each submap by **GloSSy** (FPFH descriptors → k-means dictionary → idf
  rarity). High-saliency submaps are good loop closure candidates.
- At each decision step, choose between **NextBestView** (exploration) and
  **revisit** (loop closure) based on D-optimality of the propagated pose
  covariance.

This directly addresses the "don't get lost" priority from the project goal
hierarchy and is the core contribution of the thesis.
