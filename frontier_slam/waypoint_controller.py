"""Waypoint controller node.

Drives the BlueROV2 toward the current frontier goal. Responsibilities are
split into three independent concerns that each run every control tick:

  1. XY navigation  — yaw toward the goal, surge when aligned.
  2. Depth hold     — keep the robot at a fixed depth setpoint captured on
                      the first odometry message. Goal Z is *not* used.
  3. Obstacle brake — scale surge from a forward depth-camera reading.

Why a fixed depth setpoint?
  The previous design had the frontier extractor publish gz = robot.z on each
  update. The robot drifted between updates, the extractor accepted the new
  (deeper) z as the next setpoint, and the robot sank slowly through the
  whole session (see Progress.md "Session 1 — Findings"). Locking the
  setpoint once on first odom breaks that feedback loop.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from frontier_slam.control_utils import mix_thrusters, wrap_angle, yaw_from_quat
from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'gx', 'gy', 'gz',
    'dist_m', 'hdg_err_deg', 'depth_err_m',
    'surge', 'yaw_cmd', 'heave', 'obs_m', 'blocked', 'event',
]


class WaypointController(Node):
    # P-gains
    KP_YAW   = 0.10
    KP_SURGE = 0.25
    KP_HEAVE = 0.40    # bumped from 0.30 — needs to fight persistent downward drift

    MAX_SURGE           = 0.40
    GOAL_RADIUS         = 2.0    # m
    GOAL_REACHED_TIMEOUT = 10.0  # s — clear stale goal and enter scan after this long
    SCAN_YAW            = 0.08   # rotation speed while scanning (10× KP_YAW nav max)
    OBSTACLE_SLOW_DIST  = 2.5    # m — start reducing surge below this
    OBSTACLE_STOP_DIST  = 0.8    # m — cut surge to zero below this
    ESCAPE_YAW          = 0.40   # spin rate for both sensor-blocked and position-stuck escape
    ESCAPE_DURATION     = 4.0    # s — how long to spin after a stuck trigger
    STUCK_SURGE_MIN     = 0.15   # min surge to consider "trying to move"
    STUCK_WINDOW        = 5.0    # s — detection window
    STUCK_MOVE_MIN      = 0.25   # m — minimum movement expected in STUCK_WINDOW
    CTRL_HZ             = 10.0
    LOG_EVERY_N_TICKS   = 10     # CSV row rate = CTRL_HZ / this  → 1 Hz

    def __init__(self):
        super().__init__('waypoint_controller')

        self._goal: np.ndarray | None = None
        self._pose: np.ndarray | None = None
        self._yaw  = 0.0
        self._min_front_dist = float('inf')
        self._depth_setpoint: float | None = None
        self._goal_reached_at: float | None = None
        self._stuck_ref_pos: np.ndarray | None = None
        self._stuck_ref_t:   float | None = None
        self._escape_until:  float | None = None
        self._tick = 0

        self._log = open_session_log('controller', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(PointStamped, '/frontier_slam/goal',  self._goal_cb,  1)
        self.create_subscription(Odometry,     '/StoneFish/Odometry',  self._odom_cb,  10)
        self.create_subscription(Image,        '/sensor_msgs/image_depth', self._depth_cb, 1)
        self._thrust_pub = self.create_publisher(
            Float64MultiArray, '/bluerov2/controller/thruster_setpoints_sim', 1,
        )

        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info(f'waypoint_controller ready — logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _depth_cb(self, msg: Image) -> None:
        # 32FC1 forward depth image: scan the central 40 % strip for the closest hit.
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        cx0, cx1 = data.shape[1] * 3 // 10, data.shape[1] * 7 // 10
        strip = data[:, cx0:cx1]
        valid = strip[np.isfinite(strip) & (strip > 0.1)]
        self._min_front_dist = float(valid.min()) if valid.size > 0 else float('inf')

    def _goal_cb(self, msg: PointStamped) -> None:
        # Goal Z is intentionally ignored — depth is owned by this node.
        self._goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        self._goal_reached_at = None
        self._stuck_ref_pos = None   # reset stuck window — new goal, fresh start
        self._stuck_ref_t   = None

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw  = yaw_from_quat(msg.pose.pose.orientation)
        if self._depth_setpoint is None:
            self._depth_setpoint = float(p.z)
            self.get_logger().info(
                f'depth setpoint locked at {self._depth_setpoint:.2f} m'
            )

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Control primitives
    def _heave_cmd(self) -> float:
        """Hold the depth setpoint. Negative output = upward thrust in Stonefish."""
        if self._depth_setpoint is None:
            return 0.0
        depth_err = self._pose[2] - self._depth_setpoint    # +ve = too deep
        return float(np.clip(-self.KP_HEAVE * depth_err, -1.0, 1.0))

    def _obstacle_factor(self) -> float:
        """0.0 (blocked) → 1.0 (clear). Linear ramp between STOP and SLOW distances."""
        return min(1.0, max(0.0,
            (self._min_front_dist - self.OBSTACLE_STOP_DIST) /
            (self.OBSTACLE_SLOW_DIST - self.OBSTACLE_STOP_DIST)
        ))

    def _xy_drive(self, goal_xy: np.ndarray) -> tuple:
        """Return (surge_cmd, yaw_cmd, dist_xy, heading_err, blocked)."""
        dx, dy = goal_xy - self._pose[:2]
        dist_xy = math.hypot(dx, dy)
        heading_err = wrap_angle(math.atan2(dy, dx) - self._yaw)

        yaw_cmd   = float(np.clip(self.KP_YAW * heading_err, -1.0, 1.0))
        surge_raw = self.KP_SURGE * dist_xy * max(0.0, math.cos(heading_err))
        obs       = self._obstacle_factor()
        surge_cmd = float(np.clip(surge_raw * obs, 0.0, self.MAX_SURGE))
        blocked   = obs < 0.05
        return surge_cmd, yaw_cmd, dist_xy, heading_err, blocked

    # ------------------------------------------------------------------
    # Main loop
    def _loop(self) -> None:
        self._tick += 1
        write_csv = (self._tick % self.LOG_EVERY_N_TICKS == 0)

        if self._pose is None:
            return

        heave = self._heave_cmd()

        if self._goal is None:
            self._send_thrust(0.0, self.SCAN_YAW, heave)
            if write_csv:
                self._write_csv(0.0, self.SCAN_YAW, heave, False, 'SCAN')
            return

        dist_xy = float(np.hypot(self._goal[0] - self._pose[0],
                                 self._goal[1] - self._pose[1]))
        if dist_xy < self.GOAL_RADIUS:
            now = self._t_ros()
            if self._goal_reached_at is None:
                self._goal_reached_at = now
            elif now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
                self.get_logger().info(
                    f'No new goal after {self.GOAL_REACHED_TIMEOUT:.0f}s — clearing goal, entering scan'
                )
                self._goal = None
                self._goal_reached_at = None
                # next tick will enter the _goal is None branch (scan mode)
            self._send_thrust(0.0, self.SCAN_YAW, heave)
            self.get_logger().info('Goal reached — scanning', throttle_duration_sec=2.0)
            if write_csv:
                self._write_csv(0.0, self.SCAN_YAW, heave, False, 'GOAL_REACHED',
                                dist=dist_xy, hdg_err_deg=0.0)
            return

        surge_cmd, yaw_cmd, _, heading_err, blocked = self._xy_drive(self._goal[:2])
        now = self._t_ros()
        event = ''

        # Active escape spin — finishes before resuming normal drive
        if self._escape_until is not None:
            if now < self._escape_until:
                self._send_thrust(0.0, self.ESCAPE_YAW, heave)
                if write_csv:
                    self._write_csv(0.0, self.ESCAPE_YAW, heave, blocked, 'STUCK_ESCAPE',
                                    dist=dist_xy, hdg_err_deg=math.degrees(heading_err))
                return
            self._escape_until = None
            self._stuck_ref_pos = None
            self._stuck_ref_t   = None

        # When sensor-blocked, spin hard toward goal heading instead of drifting
        if blocked:
            yaw_cmd = (1.0 if heading_err >= 0 else -1.0) * self.ESCAPE_YAW
            event = 'BLOCKED'

        # Position-stuck detection: if we push hard but barely move, trigger escape
        if surge_cmd >= self.STUCK_SURGE_MIN:
            if self._stuck_ref_pos is None:
                self._stuck_ref_pos = self._pose.copy()
                self._stuck_ref_t   = now
            else:
                moved = float(np.hypot(self._pose[0] - self._stuck_ref_pos[0],
                                       self._pose[1] - self._stuck_ref_pos[1]))
                if moved >= self.STUCK_MOVE_MIN:
                    self._stuck_ref_pos = self._pose.copy()
                    self._stuck_ref_t   = now
                elif now - self._stuck_ref_t >= self.STUCK_WINDOW:
                    self.get_logger().warn(
                        f'CTRL_STUCK: {moved:.2f}m in {self.STUCK_WINDOW:.0f}s at '
                        f'({self._pose[0]:.1f},{self._pose[1]:.1f}) — escape spin'
                    )
                    self._escape_until = now + self.ESCAPE_DURATION
                    self._stuck_ref_pos = None
                    self._stuck_ref_t   = None
                    self._send_thrust(0.0, self.ESCAPE_YAW, heave)
                    if write_csv:
                        self._write_csv(0.0, self.ESCAPE_YAW, heave, blocked, 'CTRL_STUCK',
                                        dist=dist_xy, hdg_err_deg=math.degrees(heading_err))
                    return
        else:
            self._stuck_ref_pos = None
            self._stuck_ref_t   = None

        self._send_thrust(surge_cmd, yaw_cmd, heave)

        self.get_logger().info(
            f'pos=({self._pose[0]:.1f},{self._pose[1]:.1f},{self._pose[2]:.1f}) '
            f'goal=({self._goal[0]:.1f},{self._goal[1]:.1f},{self._goal[2]:.1f}) '
            f'dist={dist_xy:.1f}m  hdg_err={math.degrees(heading_err):+.0f}°  '
            f'surge={surge_cmd:.2f}  yaw={yaw_cmd:+.3f}  heave={heave:+.2f}  '
            f'obs={self._min_front_dist:.1f}m'
            + (f'  [{event}]' if event else ''),
            throttle_duration_sec=2.0,
        )

        if write_csv:
            self._write_csv(surge_cmd, yaw_cmd, heave, blocked, event,
                            dist=dist_xy, hdg_err_deg=math.degrees(heading_err))

    # ------------------------------------------------------------------
    # Output
    def _send_thrust(self, surge: float, yaw: float, heave: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in mix_thrusters(surge, yaw, heave)]
        self._thrust_pub.publish(msg)

    def _write_csv(self, surge, yaw_cmd, heave, blocked, event,
                   dist=float('nan'), hdg_err_deg=float('nan')) -> None:
        p = self._pose
        g = self._goal
        depth_err = ((p[2] - self._depth_setpoint)
                     if self._depth_setpoint is not None else float('nan'))
        self._log.write([
            self._t_ros(),
            float(p[0]), float(p[1]), float(p[2]),
            float(g[0]) if g is not None else float('nan'),
            float(g[1]) if g is not None else float('nan'),
            float(g[2]) if g is not None else float('nan'),
            dist, hdg_err_deg, depth_err,
            surge, yaw_cmd, heave,
            self._min_front_dist, int(blocked), event,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    finally:
        node._log.close()
        node.destroy_node()
        rclpy.shutdown()
