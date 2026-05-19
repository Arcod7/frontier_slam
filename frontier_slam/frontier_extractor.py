"""
Frontier extractor — reads /projected_map, finds free-unknown boundaries,
clusters them, and publishes the nearest frontier centroid as the next goal.
"""
import csv
import os
from datetime import datetime

import numpy as np
from scipy.ndimage import binary_dilation, label

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs'
)


class FrontierExtractor(Node):
    MIN_CLUSTER_CELLS = 5    # discard clusters smaller than this
    MIN_EXPLORE_DIST  = 3.0  # m — skip frontiers the robot is already at
    SWITCH_HYSTERESIS = 3.0  # m — only switch goals if new one is this much closer
    GOAL_VANISH_DIST  = 3.0  # m — committed goal is gone if no cluster is this near it
    STUCK_TIMEOUT     = 15.0 # s — abandon goal if no progress within this window
    STUCK_MIN_PROGRESS = 0.5 # m — minimum distance closed required to not be "stuck"
    BLACKLIST_DURATION = 30.0 # s — blacklist an abandoned goal for this long
    UPDATE_HZ = 0.5          # re-evaluate every 2 s

    def __init__(self):
        super().__init__('frontier_extractor')
        self._map: OccupancyGrid | None = None
        self._robot_pos: np.ndarray | None = None  # [x, y, z] in world_ned
        self._committed: np.ndarray | None = None  # [x, y] of currently tracked goal

        # Stuck detection
        self._committed_time: float = 0.0
        self._committed_dist: float = float('inf')
        self._blacklist: list = []               # [(wx, wy, expiry_wall_time)]

        # CSV log
        os.makedirs(_LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        log_path = os.path.join(_LOG_DIR, f'{ts}_extractor.csv')
        self._log_file = open(log_path, 'w', newline='')
        self._log = csv.writer(self._log_file)
        self._log.writerow([
            't_ros', 'rx', 'ry', 'rz', 'gx', 'gy',
            'dist_m', 'clusters', 'stuck_pct', 'blacklist_n', 'event'
        ])

        self.create_subscription(OccupancyGrid, '/projected_map',    self._map_cb,  1)
        self.create_subscription(Odometry,      '/StoneFish/Odometry', self._odom_cb, 10)

        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal',      1)
        self._viz_pub  = self.create_publisher(MarkerArray,  '/frontier_slam/frontiers', 1)

        self.create_timer(1.0 / self.UPDATE_HZ, self._update)
        self.get_logger().info(f'frontier_extractor ready — logging to {log_path}')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_pos = np.array([p.x, p.y, p.z])

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_blacklisted(self, wx: float, wy: float) -> bool:
        now = self._now()
        self._blacklist = [(x, y, t) for x, y, t in self._blacklist if t > now]
        return any(
            np.hypot(wx - x, wy - y) < self.GOAL_VANISH_DIST
            for x, y, _ in self._blacklist
        )

    def _csv_row(self, gx, gy, dist, clusters, stuck_pct, event='') -> None:
        rp = self._robot_pos
        gx_s = f'{gx:.2f}' if gx == gx else ''  # nan → empty
        gy_s = f'{gy:.2f}' if gy == gy else ''
        dist_s = f'{dist:.2f}' if dist == dist else ''
        self._log.writerow([
            f'{self._now():.3f}',
            f'{rp[0]:.2f}', f'{rp[1]:.2f}', f'{rp[2]:.2f}',
            gx_s, gy_s, dist_s, clusters, stuck_pct, len(self._blacklist),
            event,
        ])
        self._log_file.flush()

    def _update(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        msg = self._map
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height

        grid    = np.array(msg.data, dtype=np.int8).reshape(h, w)
        free    = (grid == 0)
        unknown = (grid < 0)  # -1 in int8 = unknown in OccupancyGrid

        # A frontier cell is free and directly adjacent to at least one unknown cell
        frontier = free & binary_dilation(unknown, iterations=1)

        labeled, n = label(frontier)
        if n == 0:
            self.get_logger().info('No frontiers found', throttle_duration_sec=5.0)
            return

        # Collect candidate centroids (world frame)
        candidates = []
        for i in range(1, n + 1):
            rows, cols = np.where(labeled == i)
            if len(rows) < self.MIN_CLUSTER_CELLS:
                continue
            wx = ox + cols.mean() * res
            wy = oy + rows.mean() * res
            dist = np.hypot(wx - self._robot_pos[0], wy - self._robot_pos[1])
            candidates.append((wx, wy, dist, len(rows)))

        if not candidates:
            return

        # Ignore frontiers the robot is already standing in
        candidates = [c for c in candidates if c[2] >= self.MIN_EXPLORE_DIST]
        if not candidates:
            self.get_logger().info(
                'All frontiers within reach — scanning for new areas',
                throttle_duration_sec=5.0,
            )
            return

        # Filter blacklisted goals
        candidates = [c for c in candidates if not self._is_blacklisted(c[0], c[1])]
        if not candidates:
            self.get_logger().info('All candidates blacklisted — waiting', throttle_duration_sec=5.0)
            self._csv_row(float('nan'), float('nan'), float('nan'), 0, 0, 'ALL_BLACKLISTED')
            return

        candidates.sort(key=lambda c: c[2])

        # --- Stuck detection: abandon committed goal if no progress ---
        if self._committed is not None:
            current_dist_to_committed = np.hypot(
                self._committed[0] - self._robot_pos[0],
                self._committed[1] - self._robot_pos[1],
            )
            elapsed = self._now() - self._committed_time
            progress = self._committed_dist - current_dist_to_committed
            if elapsed > self.STUCK_TIMEOUT and progress < self.STUCK_MIN_PROGRESS:
                self.get_logger().warn(
                    f'STUCK: goal ({self._committed[0]:.1f},{self._committed[1]:.1f}) '
                    f'unreachable after {elapsed:.0f}s (progress={progress:.2f}m) — blacklisting'
                )
                self._csv_row(
                    self._committed[0], self._committed[1],
                    current_dist_to_committed, len(candidates),
                    min(100, int(elapsed / self.STUCK_TIMEOUT * 100)),
                    'STUCK_BLACKLIST',
                )
                self._blacklist.append((
                    self._committed[0], self._committed[1],
                    self._now() + self.BLACKLIST_DURATION,
                ))
                self._committed = None
                # Re-filter after blacklist update
                candidates = [c for c in candidates if not self._is_blacklisted(c[0], c[1])]
                if not candidates:
                    return

        # --- Goal hysteresis ---
        if self._committed is not None:
            near = [c for c in candidates
                    if np.hypot(c[0] - self._committed[0], c[1] - self._committed[1])
                    < self.GOAL_VANISH_DIST]
            if near:
                committed_dist = np.hypot(
                    self._committed[0] - self._robot_pos[0],
                    self._committed[1] - self._robot_pos[1],
                )
                if candidates[0][2] < committed_dist - self.SWITCH_HYSTERESIS:
                    gx, gy = candidates[0][0], candidates[0][1]
                    self._commit_goal(gx, gy)
                else:
                    best = min(near, key=lambda c: np.hypot(
                        c[0] - self._committed[0], c[1] - self._committed[1]))
                    gx, gy = best[0], best[1]
                    self._committed = np.array([gx, gy])  # track drift, don't reset timer
            else:
                gx, gy = candidates[0][0], candidates[0][1]
                self._commit_goal(gx, gy)
        else:
            gx, gy = candidates[0][0], candidates[0][1]
            self._commit_goal(gx, gy)

        gz = self._robot_pos[2]
        dist_to_goal = np.hypot(gx - self._robot_pos[0], gy - self._robot_pos[1])
        elapsed_on_goal = self._now() - self._committed_time
        stuck_pct = min(100, int(elapsed_on_goal / self.STUCK_TIMEOUT * 100))

        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
        self._goal_pub.publish(goal)

        self._publish_viz(candidates, gx, gy, gz)
        self.get_logger().info(
            f'robot=({self._robot_pos[0]:.1f},{self._robot_pos[1]:.1f},{self._robot_pos[2]:.1f})  '
            f'goal=({gx:.1f},{gy:.1f})  dist={dist_to_goal:.1f}m  '
            f'clusters={len(candidates)}  stuck={stuck_pct}%  '
            f'blacklist={len(self._blacklist)}'
        )
        self._csv_row(gx, gy, dist_to_goal, len(candidates), stuck_pct)

    # ------------------------------------------------------------------
    def _commit_goal(self, gx: float, gy: float) -> None:
        self._committed = np.array([gx, gy])
        self._committed_time = self._now()
        self._committed_dist = np.hypot(
            gx - self._robot_pos[0], gy - self._robot_pos[1]
        )

    def _publish_viz(self, candidates, gx, gy, gz) -> None:
        markers = MarkerArray()
        now      = self.get_clock().now().to_msg()
        lifetime = Duration(sec=3)

        # All centroids — cyan spheres
        for i, (cx, cy, _, _) in enumerate(candidates):
            m = Marker()
            m.header.stamp = now; m.header.frame_id = 'world_ned'
            m.ns = 'frontiers'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = self._robot_pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.6)
            m.lifetime = lifetime
            markers.markers.append(m)

        # Selected goal — larger red sphere
        g = Marker()
        g.header.stamp = now; g.header.frame_id = 'world_ned'
        g.ns = 'goal'; g.id = 0
        g.type = Marker.SPHERE; g.action = Marker.ADD
        g.pose.position.x = gx
        g.pose.position.y = gy
        g.pose.position.z = gz
        g.pose.orientation.w = 1.0
        g.scale.x = g.scale.y = g.scale.z = 0.8
        g.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        g.lifetime = lifetime
        markers.markers.append(g)

        self._viz_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
