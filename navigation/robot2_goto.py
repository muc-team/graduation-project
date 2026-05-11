#!/usr/bin/env python3
"""
Robot 2 — GoTo Navigator
=========================
Simple rotate-then-drive controller for point-to-point navigation.
Receives a goal pose, drives the robot there using odometry feedback.

Subscribes:
  /goal_pose  (geometry_msgs/PoseStamped)  — target location (from dashboard click)
  /odom       (nav_msgs/Odometry)          — current robot position

Publishes:
  /cmd_vel    (geometry_msgs/Twist)        — velocity commands
  /nav_status (std_msgs/String)            — navigation status for dashboard

⚠️  NO OBSTACLE AVOIDANCE — Robot 2 has no Lidar!
    Only use in areas already mapped/cleared by Robot 1.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ╔══════════════════════════════════════════════════════════════╗
# ║                  NAVIGATION TUNING                           ║
# ╚══════════════════════════════════════════════════════════════╝

# Distance & angle thresholds
GOAL_TOLERANCE    = 0.12     # meters — "arrived" when closer than this
ANGLE_TOLERANCE   = 0.15     # radians (~8°) — "aligned" when within this

# Speed limits
MAX_LINEAR_SPEED  = 0.15     # m/s — keep slow for safety (no obstacle avoidance!)
MAX_ANGULAR_SPEED = 0.40     # rad/s — turn speed limit

# Proportional gains
KP_DISTANCE       = 0.5     # linear speed proportional to distance
KP_ANGLE          = 1.2     # angular speed proportional to angle error

# Navigation states
STATE_IDLE      = 'IDLE'
STATE_ROTATING  = 'ROTATING'
STATE_DRIVING   = 'DRIVING'
STATE_ARRIVED   = 'ARRIVED'


class Robot2GoTo(Node):
    def __init__(self):
        super().__init__('robot2_goto')

        # ── Subscriptions ──
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            Twist, '/manual_cmd', self._manual_cb, 10)

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav_status', 10)

        # ── State ──
        self.state = STATE_IDLE
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.goal_x = None
        self.goal_y = None
        self.has_goal = False

        # ── Control loop at 20 Hz ──
        self.create_timer(0.05, self._navigate)

        self._publish_status(STATE_IDLE)
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Robot 2 GoTo Navigator Started')
        self.get_logger().info(f'  Goal tolerance : {GOAL_TOLERANCE} m')
        self.get_logger().info(f'  Max speed      : {MAX_LINEAR_SPEED} m/s')
        self.get_logger().info('  ⚠️  No obstacle avoidance!')
        self.get_logger().info('═══════════════════════════════════════')

    # ─────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        """Update current robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)

    def _goal_cb(self, msg: PoseStamped):
        """Receive a new navigation goal."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        self.state = STATE_ROTATING

        self.get_logger().info(
            f'📍 New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self._publish_status(STATE_ROTATING)

    def _manual_cb(self, msg: Twist):
        """If user manually drives or stops, cancel the goal."""
        if self.has_goal:
            self.get_logger().info('🛑 Manual override! Cancelling goal.')
            self.cancel_goal()

    # ─────────────────────────────────────
    # NAVIGATION LOOP
    # ─────────────────────────────────────

    def _navigate(self):
        """Main navigation control loop (20 Hz)."""
        if not self.has_goal:
            return

        # Compute distance and angle to goal
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)
        angle_to_goal = math.atan2(dy, dx)

        # Angle error (normalized to [-π, π])
        angle_error = angle_to_goal - self.robot_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        # ── Phase 1: ARRIVED ──
        if distance < GOAL_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if self.state != STATE_ARRIVED:
                self.state = STATE_ARRIVED
                self._publish_status(STATE_ARRIVED)
                self.get_logger().info(
                    f'✅ Arrived at ({self.goal_x:.2f}, {self.goal_y:.2f})!')
            self.has_goal = False
            return

        # ── Phase 2: ROTATE to face goal ──
        if abs(angle_error) > ANGLE_TOLERANCE:
            self.state = STATE_ROTATING
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_ANGULAR_SPEED,
                                min(MAX_ANGULAR_SPEED,
                                    KP_ANGLE * angle_error))
            self.cmd_pub.publish(cmd)
            self._publish_status(STATE_ROTATING)
            return

        # ── Phase 3: DRIVE toward goal ──
        self.state = STATE_DRIVING

        # Linear speed proportional to distance, clamped
        linear = min(MAX_LINEAR_SPEED, KP_DISTANCE * distance)

        # Small angular correction while driving to stay on course
        angular = max(-MAX_ANGULAR_SPEED * 0.5,
                      min(MAX_ANGULAR_SPEED * 0.5,
                          KP_ANGLE * 0.5 * angle_error))

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        self._publish_status(STATE_DRIVING)

    # ─────────────────────────────────────
    # STATUS PUBLISHING
    # ─────────────────────────────────────

    def _publish_status(self, status: str):
        """Publish navigation status for dashboard."""
        msg = String()
        if status == STATE_ARRIVED:
            msg.data = f'ARRIVED:{self.goal_x:.2f},{self.goal_y:.2f}'
        elif status == STATE_ROTATING:
            msg.data = f'ROTATING:{self.goal_x:.2f},{self.goal_y:.2f}'
        elif status == STATE_DRIVING:
            dx = self.goal_x - self.robot_x
            dy = self.goal_y - self.robot_y
            dist = math.sqrt(dx * dx + dy * dy)
            msg.data = f'DRIVING:{dist:.2f}m'
        else:
            msg.data = 'IDLE'
        self.status_pub.publish(msg)

    def cancel_goal(self):
        """Stop the robot and cancel current goal."""
        self.has_goal = False
        self.state = STATE_IDLE
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self._publish_status(STATE_IDLE)
        self.get_logger().info('🛑 Goal cancelled')


def main(args=None):
    rclpy.init(args=args)
    node = Robot2GoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
