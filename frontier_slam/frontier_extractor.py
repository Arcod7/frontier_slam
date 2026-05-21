"""Frontier extractor node.

Subscribes to /projected_map (the 2-D OctoMap projection) and /StoneFish/Odometry.
On each tick it:
  1. Finds free-cell ↔ unknown-cell boundaries (frontiers) and clusters them.
  2. Lets a GoalManager pick the next goal, with hysteresis, stuck detection,
     and a timed blacklist for unreachable goals.
  3. Publishes the goal on /frontier_slam/goal and an RViz MarkerArray on
     /frontier_slam/frontiers.

The goal's Z is set to the robot's current Z — but only for marker placement.
The waypoint controller maintains its own fixed depth setpoint and ignores
goal.point.z.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from frontier_slam.control_utils import yaw_from_quat
from frontier_slam.frontier_detection import find_frontier_clusters
from frontier_slam.goal_manager import GoalManager
from frontier_slam.path_planner import PAD_CELLS, CostGrid, build_cost_grid, find_path
from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'gx', 'gy',
    'dist_m', 'clusters', 'stuck_pct', 'blacklist_n',
    'free_cells', 'occ_cells', 'mapped_cells', 'event',
]


class FrontierExtractor(Node):
    MIN_CLUSTER_CELLS = 5
    UPDATE_HZ         = 0.5
    REPLAN_HZ         = 3.0
    REPLAN_FAIL_MAX   = 6    # consecutive A* failures before blacklisting goal as unreachable

    def __init__(self):
        super().__init__('frontier_extractor')

        self._map: OccupancyGrid | None = None
        self._robot_pos: np.ndarray | None = None
        self._current_goal_xy: np.ndarray | None = None
        self._current_path: list = []
        self._cg: CostGrid | None = None
        self._robot_yaw: float = 0.0
        self._robot_speed: float = 0.0
        self._last_stuck_pct: int = 0
        self._astar_fail_count: int = 0
        self._astar_fail_goal: np.ndarray | None = None

        self._goals = GoalManager(
            min_explore_dist=3.0,
            goal_vanish_dist=3.0,
            goal_radius=2.0,
            stuck_timeout=30.0,
            stuck_min_progress=0.5,
            blacklist_duration=30.0,
            arrival_blacklist_duration=20.0,
        )

        self._log = open_session_log('extractor', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(OccupancyGrid, '/projected_map',     self._map_cb,  1)
        self.create_subscription(Odometry,      '/StoneFish/Odometry', self._odom_cb, 10)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal',      1)
        self._path_pub = self.create_publisher(Path,         '/frontier_slam/path',      1)
        self._viz_pub          = self.create_publisher(MarkerArray,   '/frontier_slam/frontiers',    1)
        self._inflated_map_pub = self.create_publisher(OccupancyGrid, '/frontier_slam/inflated_map', 1)
        self._debug_img_pub    = self.create_publisher(Image,         '/frontier_slam/debug_image',  1)

        self.create_timer(1.0 / self.UPDATE_HZ,  self._update)
        self.create_timer(1.0 / self.REPLAN_HZ,  self._replan)
        self.get_logger().info(f'frontier_extractor ready — logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_pos   = np.array([p.x, p.y, p.z])
        self._robot_yaw   = yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self._robot_speed = math.hypot(v.x, v.y)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Map statistics
    def _map_stats(self) -> tuple[int, int, int]:
        """Return (free_cells, occupied_cells, mapped_cells) from current map."""
        data = np.asarray(self._map.data, dtype=np.int8)
        free = int(np.sum(data == 0))
        occ  = int(np.sum(data == 100))
        return free, occ, free + occ

    # ------------------------------------------------------------------
    # Main loop
    def _update(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        free_cells, occ_cells, mapped_cells = self._map_stats()

        clusters = find_frontier_clusters(self._map, self.MIN_CLUSTER_CELLS)
        if not clusters:
            self.get_logger().info('No frontiers found', throttle_duration_sec=5.0)
            return

        # Tag each cluster with its distance from the robot.
        for c in clusters:
            c.distance = float(np.hypot(c.wx - self._robot_pos[0],
                                        c.wy - self._robot_pos[1]))

        selection = self._goals.select(clusters, self._robot_pos[:2], self._now())

        if selection is None:
            self.get_logger().info(
                'All frontiers within reach — scanning for new areas',
                throttle_duration_sec=5.0,
            )
            return

        # Surface the stuck event for the human-readable ROS log.
        if selection.event == 'STUCK_BLACKLIST' and selection.stuck_goal is not None:
            sx, sy, sec, prog = selection.stuck_goal
            self.get_logger().warn(
                f'STUCK: goal ({sx:.1f},{sy:.1f}) unreachable after {sec:.0f}s '
                f'(progress={prog:.2f}m) — blacklisting'
            )

        if selection.event == 'ALL_BLACKLISTED' or math.isnan(selection.gx):
            self.get_logger().info(
                'All candidates blacklisted — waiting', throttle_duration_sec=5.0,
            )
            self._current_goal_xy = None
            self._current_path    = []
            self._write_csv(float('nan'), float('nan'), float('nan'),
                            len(clusters), 0,
                            free_cells, occ_cells, mapped_cells, selection.event or 'ALL_BLACKLISTED')
            return

        self._last_stuck_pct = selection.stuck_pct
        self._publish_goal(selection.gx, selection.gy, clusters)
        dist = float(np.hypot(selection.gx - self._robot_pos[0],
                              selection.gy - self._robot_pos[1]))
        self.get_logger().info(
            f'robot=({self._robot_pos[0]:.1f},{self._robot_pos[1]:.1f},{self._robot_pos[2]:.1f})  '
            f'goal=({selection.gx:.1f},{selection.gy:.1f})  dist={dist:.1f}m  '
            f'clusters={len(clusters)}  stuck={selection.stuck_pct}%  '
            f'blacklist={self._goals.blacklist_size}  '
            f'mapped={mapped_cells}(free={free_cells},occ={occ_cells})'
        )
        self._write_csv(selection.gx, selection.gy, dist,
                        len(clusters), selection.stuck_pct,
                        free_cells, occ_cells, mapped_cells, selection.event)

    # ------------------------------------------------------------------
    # Publishing
    def _publish_goal(self, gx: float, gy: float, clusters: list) -> None:
        self._current_goal_xy = np.array([gx, gy])
        gz = float(self._robot_pos[2])   # only used for marker placement
        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
        self._goal_pub.publish(goal)
        self._publish_viz(clusters, gx, gy, gz)

    def _replan(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        self._cg = build_cost_grid(self._map)

        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'world_ned'
        if self._current_goal_xy is not None and not np.any(np.isnan(self._current_goal_xy)):
            # Reset failure counter when the goal changes.
            if (self._astar_fail_goal is None or
                    np.hypot(self._current_goal_xy[0] - self._astar_fail_goal[0],
                             self._current_goal_xy[1] - self._astar_fail_goal[1]) > 1.0):
                self._astar_fail_count = 0
                self._astar_fail_goal  = self._current_goal_xy.copy()

            waypoints = find_path(self._cg, self._robot_pos[:2], self._current_goal_xy)
            self._current_path = waypoints

            if waypoints:
                self._astar_fail_count = 0
                gz = float(self._robot_pos[2])
                for wx, wy in waypoints:
                    ps = PoseStamped()
                    ps.header = path.header
                    ps.pose.position.x = wx
                    ps.pose.position.y = wy
                    ps.pose.position.z = gz
                    ps.pose.orientation.w = 1.0
                    path.poses.append(ps)
            else:
                self._astar_fail_count += 1
                gxy = self._current_goal_xy
                if self._astar_fail_count >= self.REPLAN_FAIL_MAX:
                    self.get_logger().warn(
                        f'A* failed {self.REPLAN_FAIL_MAX}× for '
                        f'({gxy[0]:.1f},{gxy[1]:.1f}) — blacklisting as unreachable'
                    )
                    self._goals.mark_unreachable(gxy, self._now())
                    self._current_goal_xy  = None
                    self._current_path     = []
                    self._astar_fail_count = 0
                    self._astar_fail_goal  = None
                else:
                    self.get_logger().warn(
                        f'A* found no path to ({gxy[0]:.1f},{gxy[1]:.1f}) '
                        f'({self._astar_fail_count}/{self.REPLAN_FAIL_MAX})'
                    )
        self._path_pub.publish(path)
        self._publish_inflated_map()
        self._publish_debug_image()

    def _publish_viz(self, clusters: list, gx: float, gy: float, gz: float) -> None:
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=3)

        for i, c in enumerate(clusters):
            markers.markers.append(self._sphere(
                ns='frontiers', mid=i, x=c.wx, y=c.wy, z=self._robot_pos[2],
                scale=0.4, rgba=(0.0, 1.0, 1.0, 0.6),
                stamp=now, lifetime=lifetime,
            ))
        markers.markers.append(self._sphere(
            ns='goal', mid=0, x=gx, y=gy, z=gz,
            scale=0.8, rgba=(1.0, 0.0, 0.0, 0.9),
            stamp=now, lifetime=lifetime,
        ))
        # Arrow pointing at the goal — tail is 2 m behind the goal along the approach vector,
        # head is at the goal itself, showing the heading the robot needs to hold.
        dx, dy = gx - self._robot_pos[0], gy - self._robot_pos[1]
        dist = math.hypot(dx, dy)
        if dist > 0.5:
            arrow_len = min(2.0, dist * 0.8)
            ux, uy = dx / dist, dy / dist
            arrow = Marker()
            arrow.header.stamp    = now
            arrow.header.frame_id = 'world_ned'
            arrow.ns      = 'goal_arrow'
            arrow.id      = 0
            arrow.type    = Marker.ARROW
            arrow.action  = Marker.ADD
            arrow.points  = [
                Point(x=gx - ux * arrow_len, y=gy - uy * arrow_len, z=gz),
                Point(x=gx, y=gy, z=gz),
            ]
            arrow.scale.x = 0.15   # shaft diameter
            arrow.scale.y = 0.35   # head diameter
            arrow.color   = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            arrow.lifetime = lifetime
            markers.markers.append(arrow)
        self._viz_pub.publish(markers)

    @staticmethod
    def _sphere(*, ns, mid, x, y, z, scale, rgba, stamp, lifetime) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'world_ned'
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        m.lifetime = lifetime
        return m

    # ------------------------------------------------------------------
    # Debug visualisation
    def _publish_inflated_map(self) -> None:
        if self._map is None or self._cg is None:
            return
        info = self._map.info
        if self._cg.raw.shape != (info.height, info.width):
            return
        raw = self._cg.raw
        out = raw.copy()
        p   = PAD_CELLS
        out[self._cg.plan_zone[p:-p, p:-p]    & (raw != 100)] = 35  # planning zone → 35
        out[self._cg.soft_zone[p:-p, p:-p]    & (raw != 100)] = 50  # soft zone → 50
        out[self._cg.hard_blocked[p:-p, p:-p] & (raw != 100)] = 75  # hard zone → 75
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.info            = self._map.info
        msg.data            = out.flatten().tolist()
        self._inflated_map_pub.publish(msg)

    @staticmethod
    def _draw_line(img: np.ndarray, r0: int, c0: int, r1: int, c1: int,
                   color: tuple) -> None:
        """Draw a 1-pixel line by interpolation (no external deps)."""
        n = max(abs(r1 - r0), abs(c1 - c0), 1)
        rs = np.round(np.linspace(r0, r1, n)).astype(int)
        cs = np.round(np.linspace(c0, c1, n)).astype(int)
        h, w = img.shape[:2]
        ok = (rs >= 0) & (rs < h) & (cs >= 0) & (cs < w)
        img[rs[ok], cs[ok]] = color

    def _robot_arrow_color(self) -> tuple:
        if self._current_goal_xy is None:
            return (0, 200, 200)   # cyan  = scanning / no valid goal
        if self._last_stuck_pct >= 100:
            return (220, 100, 0)   # orange = maxed stuck
        if self._last_stuck_pct >= 50:
            return (220, 220, 0)   # yellow = approaching stuck timeout
        return (50, 150, 255)      # blue  = navigating normally

    def _publish_debug_image(self) -> None:
        if self._map is None or self._robot_pos is None:
            return
        info = self._map.info
        h, w = info.height, info.width
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        raw = (self._cg.raw if (self._cg is not None and self._cg.raw.shape == (h, w))
               else np.asarray(self._map.data, dtype=np.int8).reshape(h, w))
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[raw == -1] = (80,  80,  80)   # unknown
        img[raw == 0]  = (210, 210, 210)  # free
        img[raw == 100]= (20,  20,  20)   # occupied

        if self._cg is not None and self._cg.raw.shape == (h, w):
            p = PAD_CELLS
            img[self._cg.plan_zone[p:-p, p:-p]    & (raw != 100)] = (180, 160, 60)  # planning zone
            img[self._cg.soft_zone[p:-p, p:-p]    & (raw != 100)] = (200, 100, 50)  # soft zone
            img[self._cg.hard_blocked[p:-p, p:-p] & (raw != 100)] = (150, 30,  30)  # hard zone

        # Path as connected line segments (green)
        if self._current_path:
            pts = [(int((wy - oy) / res), int((wx - ox) / res))
                   for wx, wy in self._current_path]
            for i in range(len(pts) - 1):
                self._draw_line(img, pts[i][0], pts[i][1],
                                pts[i + 1][0], pts[i + 1][1], (0, 220, 0))

        # Goal: red cross
        if self._current_goal_xy is not None:
            gc = int((self._current_goal_xy[0] - ox) / res)
            gr = int((self._current_goal_xy[1] - oy) / res)
            if 0 <= gr < h and 0 <= gc < w:
                for d in range(-4, 5):
                    if 0 <= gr + d < h: img[gr + d, gc] = (220, 50, 50)
                    if 0 <= gc + d < w: img[gr, gc + d] = (220, 50, 50)

        # Robot: arrow aligned to heading, length ∝ speed
        # Pre-flip coords: col↔X(North/right), row↔Y(East/down-before-flip→up-after)
        # dc=cos(yaw), dr=sin(yaw) maps correctly after flipud.
        rc = int((self._robot_pos[0] - ox) / res)
        rr = int((self._robot_pos[1] - oy) / res)
        arm = max(4, int(4 + self._robot_speed * 20))   # pixels; ~10px at MAX_SURGE
        dc  = int(round(math.cos(self._robot_yaw) * arm))
        dr  = int(round(math.sin(self._robot_yaw) * arm))
        color = self._robot_arrow_color()
        if 0 <= rr < h and 0 <= rc < w:
            self._draw_line(img, rr, rc, rr + dr, rc + dc, color)
            # 3×3 body dot at tail
            img[max(0, rr - 1):min(h, rr + 2), max(0, rc - 1):min(w, rc + 2)] = color
            # 3×3 arrowhead dot at tip
            tr, tc = rr + dr, rc + dc
            if 0 <= tr < h and 0 <= tc < w:
                img[max(0, tr - 1):min(h, tr + 2), max(0, tc - 1):min(w, tc + 2)] = color

        # Flip vertical axis (OccupancyGrid row-0 = south, image row-0 = top)
        out = np.flipud(img)

        # 4-pixel status bar at top — colour encodes robot state
        bar = ((0, 150, 150) if self._current_goal_xy is None else
               (200, 80,  0) if self._last_stuck_pct >= 100      else
               (200, 200, 0) if self._last_stuck_pct >= 50       else
               (0,  120,  0))
        out[0:4, :] = bar

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.height          = h
        msg.width           = w
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = False
        msg.step            = w * 3
        msg.data            = out.tobytes()
        self._debug_img_pub.publish(msg)

    # ------------------------------------------------------------------
    # Logging
    def _write_csv(self, gx, gy, dist, n_clusters, stuck_pct,
                   free_cells, occ_cells, mapped_cells, event) -> None:
        rp = self._robot_pos
        self._log.write([
            self._now(),
            float(rp[0]), float(rp[1]), float(rp[2]),
            float(gx), float(gy), float(dist),
            n_clusters, stuck_pct, self._goals.blacklist_size,
            free_cells, occ_cells, mapped_cells, event,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExtractor()
    try:
        rclpy.spin(node)
    finally:
        node._log.close()
        node.destroy_node()
        rclpy.shutdown()
