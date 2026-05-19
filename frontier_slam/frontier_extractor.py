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
import os

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from frontier_slam.frontier_detection import find_frontier_clusters
from frontier_slam.goal_manager import GoalManager
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

    def __init__(self):
        super().__init__('frontier_extractor')

        self._map: OccupancyGrid | None = None
        self._robot_pos: np.ndarray | None = None

        self._goals = GoalManager(
            min_explore_dist=3.0,
            switch_hysteresis=3.0,
            goal_vanish_dist=3.0,
            stuck_timeout=15.0,
            stuck_min_progress=0.5,
            blacklist_duration=30.0,
        )

        self._log = open_session_log('extractor', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(OccupancyGrid, '/projected_map',     self._map_cb,  1)
        self.create_subscription(Odometry,      '/StoneFish/Odometry', self._odom_cb, 10)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal',      1)
        self._viz_pub  = self.create_publisher(MarkerArray,  '/frontier_slam/frontiers', 1)

        self.create_timer(1.0 / self.UPDATE_HZ, self._update)
        self.get_logger().info(f'frontier_extractor ready — logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_pos = np.array([p.x, p.y, p.z])

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

        if selection.event == 'ALL_BLACKLISTED':
            self.get_logger().info(
                'All candidates blacklisted — waiting', throttle_duration_sec=5.0,
            )
            self._write_csv(float('nan'), float('nan'), float('nan'),
                            len(clusters), 0,
                            free_cells, occ_cells, mapped_cells, 'ALL_BLACKLISTED')
            return

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
        gz = float(self._robot_pos[2])   # only used for marker placement
        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
        self._goal_pub.publish(goal)
        self._publish_viz(clusters, gx, gy, gz)

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
