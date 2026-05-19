"""
Frontier extractor — reads /projected_map, finds free-unknown boundaries,
clusters them, and publishes the nearest frontier centroid as the next goal.
"""
import numpy as np
from scipy.ndimage import binary_dilation, label

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


class FrontierExtractor(Node):
    MIN_CLUSTER_CELLS = 5   # discard clusters smaller than this
    MIN_EXPLORE_DIST  = 3.0 # m — skip frontiers the robot is already at
    UPDATE_HZ = 0.5         # re-evaluate every 2 s

    def __init__(self):
        super().__init__('frontier_extractor')
        self._map: OccupancyGrid | None = None
        self._robot_pos: np.ndarray | None = None  # [x, y, z] in world_ned

        self.create_subscription(OccupancyGrid, '/projected_map',    self._map_cb,  1)
        self.create_subscription(Odometry,      '/StoneFish/Odometry', self._odom_cb, 10)

        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal',      1)
        self._viz_pub  = self.create_publisher(MarkerArray,  '/frontier_slam/frontiers', 1)

        self.create_timer(1.0 / self.UPDATE_HZ, self._update)
        self.get_logger().info('frontier_extractor ready')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_pos = np.array([p.x, p.y, p.z])

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    # ------------------------------------------------------------------
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

        # Nearest centroid that requires actual movement
        candidates.sort(key=lambda c: c[2])
        gx, gy, _, _ = candidates[0]
        gz = self._robot_pos[2]  # maintain current depth (2-D frontier on projected map)

        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
        self._goal_pub.publish(goal)

        self._publish_viz(candidates, gx, gy, gz)
        self.get_logger().info(
            f'Goal → ({gx:.1f}, {gy:.1f}, {gz:.1f}) m  |  {len(candidates)} clusters'
        )

    # ------------------------------------------------------------------
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
