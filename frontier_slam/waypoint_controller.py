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
import csv
import math
import os
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs'
)


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class WaypointController(Node):
    # P-gains
    KP_YAW   = 0.04   # 10× slower than before — smooth heading changes
    KP_SURGE = 0.25
    KP_HEAVE = 0.3

    MAX_SURGE          = 0.40
    GOAL_RADIUS        = 2.0    # m
    SCAN_YAW           = 0.008  # slow scan rotation
    OBSTACLE_SLOW_DIST = 2.5    # m — start reducing surge below this
    OBSTACLE_STOP_DIST = 0.8    # m — cut surge to zero below this
    CTRL_HZ            = 10.0

    def __init__(self):
        super().__init__('waypoint_controller')
        self._goal: np.ndarray | None = None   # [x, y, z]
        self._pose: np.ndarray | None = None   # [x, y, z]
        self._yaw  = 0.0
        self._min_front_dist = float('inf')    # nearest obstacle ahead (m)
        self._tick = 0                          # for CSV subsampling

        # CSV log (write at 1 Hz = every 10th tick)
        os.makedirs(_LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        log_path = os.path.join(_LOG_DIR, f'{ts}_controller.csv')
        self._log_file = open(log_path, 'w', newline='')
        self._log = csv.writer(self._log_file)
        self._log.writerow([
            't_ros', 'rx', 'ry', 'rz', 'gx', 'gy', 'gz',
            'dist_m', 'hdg_err_deg', 'surge', 'yaw_cmd', 'heave',
            'obs_m', 'blocked', 'event'
        ])

        self.create_subscription(PointStamped, '/frontier_slam/goal',  self._goal_cb,  1)
        self.create_subscription(Odometry,     '/StoneFish/Odometry',  self._odom_cb,  10)
        self.create_subscription(Image,        '/sensor_msgs/image_depth', self._depth_cb, 1)

        self._thrust_pub = self.create_publisher(
            Float64MultiArray, '/bluerov2/controller/thruster_setpoints_sim', 1
        )
        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info(f'waypoint_controller ready — logging to {log_path}')

    # ------------------------------------------------------------------
    def _depth_cb(self, msg: Image) -> None:
        # Parse 32FC1 forward depth image; check central 40 % strip for obstacles
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        cx0, cx1 = data.shape[1] * 3 // 10, data.shape[1] * 7 // 10
        strip = data[:, cx0:cx1]
        valid = strip[np.isfinite(strip) & (strip > 0.1)]
        self._min_front_dist = float(valid.min()) if valid.size > 0 else float('inf')

    def _goal_cb(self, msg: PointStamped) -> None:
        self._goal = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw  = _yaw_from_quat(msg.pose.pose.orientation)

    # ------------------------------------------------------------------
    def _csv_row(self, surge, yaw_cmd, heave, blocked, event='',
                 dist=float('nan'), hdg_err_deg=float('nan'),
                 gx=float('nan'), gy=float('nan'), gz=float('nan')) -> None:
        p = self._pose
        obs = self._min_front_dist
        obs_s = f'{obs:.2f}' if math.isfinite(obs) else 'inf'
        def _f(v): return f'{v:.2f}' if v == v else ''  # nan → empty string
        self._log.writerow([
            f'{self.get_clock().now().nanoseconds * 1e-9:.3f}',
            f'{p[0]:.2f}', f'{p[1]:.2f}', f'{p[2]:.2f}',
            _f(gx), _f(gy), _f(gz),
            _f(dist), f'{hdg_err_deg:.1f}' if hdg_err_deg == hdg_err_deg else '',
            f'{surge:.3f}', f'{yaw_cmd:.4f}', f'{heave:.3f}',
            obs_s, int(blocked), event,
        ])
        self._log_file.flush()

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        self._tick += 1
        write_csv = (self._tick % 10 == 0)  # 1 Hz

        if self._pose is None:
            return
        if self._goal is None:
            self._mix_and_send(0.0, self.SCAN_YAW, 0.0)
            if write_csv:
                self._csv_row(0.0, self.SCAN_YAW, 0.0, False, 'SCAN')
            return

        dx, dy, dz = self._goal - self._pose
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.hypot(dist_xy, dz)

        if dist_3d < self.GOAL_RADIUS:
            heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))
            self._mix_and_send(0.0, self.SCAN_YAW, heave_cmd)
            self.get_logger().info('Goal reached — scanning', throttle_duration_sec=2.0)
            if write_csv:
                self._csv_row(
                    0.0, self.SCAN_YAW, heave_cmd, False, 'GOAL_REACHED',
                    dist=dist_3d, hdg_err_deg=0.0,
                    gx=self._goal[0], gy=self._goal[1], gz=self._goal[2],
                )
            return

        desired_yaw = math.atan2(dy, dx)
        heading_err = _wrap(desired_yaw - self._yaw)

        yaw_cmd = float(np.clip(self.KP_YAW * heading_err, -1.0, 1.0))
        surge_raw = self.KP_SURGE * dist_xy * max(0.0, math.cos(heading_err))
        obs_factor = min(1.0, max(0.0,
            (self._min_front_dist - self.OBSTACLE_STOP_DIST) /
            (self.OBSTACLE_SLOW_DIST - self.OBSTACLE_STOP_DIST)
        ))
        surge_cmd = float(np.clip(surge_raw * obs_factor, 0.0, self.MAX_SURGE))
        heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))

        blocked = obs_factor < 0.05
        self.get_logger().info(
            f'pos=({self._pose[0]:.1f},{self._pose[1]:.1f},{self._pose[2]:.1f}) '
            f'goal=({self._goal[0]:.1f},{self._goal[1]:.1f},{self._goal[2]:.1f}) '
            f'dist={dist_3d:.1f}m  hdg_err={math.degrees(heading_err):+.0f}°  '
            f'surge={surge_cmd:.2f}  yaw={yaw_cmd:+.3f}  heave={heave_cmd:+.2f}  '
            f'obs={self._min_front_dist:.1f}m'
            + ('  [BLOCKED]' if blocked else ''),
            throttle_duration_sec=2.0,
        )

        if write_csv:
            self._csv_row(
                surge_cmd, yaw_cmd, heave_cmd, blocked,
                'BLOCKED' if blocked else '',
                dist=dist_3d, hdg_err_deg=math.degrees(heading_err),
                gx=self._goal[0], gy=self._goal[1], gz=self._goal[2],
            )

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
