"""
Waypoint controller — drives the BlueROV2 toward the frontier goal.

Subscribes to /frontier_slam/goal (PointStamped) and /StoneFish/Odometry.
Publishes Float64MultiArray to /bluerov2/controller/thruster_setpoints_sim.

Behaviour:
  1. Large heading error → yaw in place first.
  2. Roughly aligned    → surge forward + small heading correction.
  3. Depth error        → heave independently on both vertical thrusters.

Thruster mixing (BlueROV2 vectored, NED body frame, thrusters in scn order):
  Index  Name        Formula
  0      FrontRight   surge - yaw
  1      FrontLeft    surge + yaw
  2      BackRight   -surge + yaw
  3      BackLeft    -surge - yaw
  4      VertFront    heave
  5      VertBack     heave
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class WaypointController(Node):
    # P-gains
    KP_YAW   = 0.4
    KP_SURGE = 0.25
    KP_HEAVE = 0.3

    MAX_SURGE   = 0.45  # hard cap on forward speed
    GOAL_RADIUS = 2.0   # m
    SCAN_YAW    = 0.08  # slow rotation while waiting for next frontier
    CTRL_HZ     = 10.0

    def __init__(self):
        super().__init__('waypoint_controller')
        self._goal: np.ndarray | None = None   # [x, y, z]
        self._pose: np.ndarray | None = None   # [x, y, z]
        self._yaw  = 0.0

        self.create_subscription(PointStamped, '/frontier_slam/goal',    self._goal_cb, 1)
        self.create_subscription(Odometry,     '/StoneFish/Odometry',    self._odom_cb, 10)

        self._thrust_pub = self.create_publisher(
            Float64MultiArray, '/bluerov2/controller/thruster_setpoints_sim', 1
        )
        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info('waypoint_controller ready')

    # ------------------------------------------------------------------
    def _goal_cb(self, msg: PointStamped) -> None:
        self._goal = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw  = _yaw_from_quat(msg.pose.pose.orientation)

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        if self._pose is None:
            return
        if self._goal is None:
            # No frontier yet — hold depth and scan to build initial map
            self._mix_and_send(0.0, self.SCAN_YAW, 0.0)
            return

        dx, dy, dz = self._goal - self._pose
        dist_xy = math.hypot(dx, dy)

        if math.hypot(dist_xy, dz) < self.GOAL_RADIUS:
            # Hold depth + slow yaw scan while waiting for next frontier
            heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))
            self._mix_and_send(0.0, self.SCAN_YAW, heave_cmd)
            self.get_logger().info('Goal reached — scanning', throttle_duration_sec=2.0)
            return

        desired_yaw = math.atan2(dy, dx)
        heading_err = _wrap(desired_yaw - self._yaw)

        yaw_cmd = float(np.clip(self.KP_YAW * heading_err, -1.0, 1.0))
        # Surge scales with heading alignment via cos — smooth curved approach,
        # naturally zero when facing away, full when aligned, no hard threshold.
        surge_cmd = float(np.clip(
            self.KP_SURGE * dist_xy * max(0.0, math.cos(heading_err)),
            0.0, self.MAX_SURGE,
        ))
        heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))

        self._mix_and_send(surge_cmd, yaw_cmd, heave_cmd)

    def _mix_and_send(self, surge: float, yaw: float, heave: float) -> None:
        raw = [
            surge - yaw,   # T0 FrontRight
            surge + yaw,   # T1 FrontLeft
           -surge + yaw,   # T2 BackRight
           -surge - yaw,   # T3 BackLeft
            heave,         # T4 VertFront
            heave,         # T5 VertBack
        ]
        # Rescale so no value exceeds ±1 (preserves ratios)
        peak = max(abs(v) for v in raw)
        if peak > 1.0:
            raw = [v / peak for v in raw]
        self._send(raw)

    def _send(self, values) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self._thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
