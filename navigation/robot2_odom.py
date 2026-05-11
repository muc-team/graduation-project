#!/usr/bin/env python3
"""
Robot 2 — Dead-Reckoning Odometry Node
=======================================
Fuses encoder ticks + IMU gyroscope to compute (x, y, θ) position.
Publishes nav_msgs/Odometry on /odom at 50 Hz.

Subscribes:
  /encoders      (std_msgs/Int32MultiArray)  — cumulative encoder ticks
  /imu/data_raw  (sensor_msgs/Imu)           — raw IMU readings

Publishes:
  /odom          (nav_msgs/Odometry)          — robot pose in map frame

╔══════════════════════════════════════════════════════════════╗
║  CALIBRATION REQUIRED — Measure these from YOUR robot:      ║
║    WHEEL_DIAMETER  = diameter of one wheel (meters)         ║
║    TICKS_PER_REV   = encoder ticks for 1 full revolution    ║
║    WHEEL_BASE      = left↔right wheel center distance (m)   ║
╚══════════════════════════════════════════════════════════════╝
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros


# ╔══════════════════════════════════════════════════════════════╗
# ║                  ROBOT PHYSICAL CONSTANTS                    ║
# ║         ⚠️  CHANGE THESE TO MATCH YOUR ROBOT  ⚠️             ║
# ╚══════════════════════════════════════════════════════════════╝

WHEEL_DIAMETER = 0.065      # 65 mm wheels (measure yours!)
TICKS_PER_REV  = 330        # Ticks per full wheel revolution (measure!)
WHEEL_BASE     = 0.23       # Distance between left & right wheels (meters)

# Derived
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV

# Complementary filter weight for heading
# 0.0 = pure IMU gyro, 1.0 = pure encoders
ENCODER_HEADING_WEIGHT = 0.3


class Robot2Odom(Node):
    def __init__(self):
        super().__init__('robot2_odom')

        # ── Subscriptions ──
        self.create_subscription(
            Int32MultiArray, '/encoders', self._enc_cb, 10)
        self.create_subscription(
            Imu, '/imu/data_raw', self._imu_cb, 10)

        # ── Publisher ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # ── TF Broadcaster ──
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── State ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder tracking
        self.prev_enc = None          # [FL, RL, FR, RR] — previous tick values
        self.last_enc_time = None

        # IMU tracking
        self.gyro_z = 0.0             # Latest gyro reading (rad/s)

        # Timer for publishing at 50 Hz
        self.create_timer(0.02, self._publish_odom)

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Robot 2 Odometry Node Started')
        self.get_logger().info(f'  Wheel ∅ = {WHEEL_DIAMETER*100:.1f} cm')
        self.get_logger().info(f'  Ticks/Rev = {TICKS_PER_REV}')
        self.get_logger().info(f'  Wheel Base = {WHEEL_BASE*100:.1f} cm')
        self.get_logger().info(f'  m/tick = {METERS_PER_TICK:.6f}')
        self.get_logger().info('═══════════════════════════════════════')

    def _imu_cb(self, msg: Imu):
        """Store latest gyroscope Z reading (already in rad/s from bridge)."""
        self.gyro_z = msg.angular_velocity.z

    def _enc_cb(self, msg: Int32MultiArray):
        """Process encoder data: [Front-Left, Rear-Left, Front-Right, Rear-Right]."""
        if len(msg.data) < 4:
            return

        enc = list(msg.data)  # [FL, RL, FR, RR]
        now = self.get_clock().now()

        if self.prev_enc is None:
            # First reading — initialize, no motion yet
            self.prev_enc = enc
            self.last_enc_time = now
            return

        # ── Compute deltas ──
        d_fl = enc[0] - self.prev_enc[0]
        d_rl = enc[1] - self.prev_enc[1]
        d_fr = enc[2] - self.prev_enc[2]
        d_rr = enc[3] - self.prev_enc[3]

        # Average left and right sides
        d_left  = (d_fl + d_rl) / 2.0
        d_right = (d_fr + d_rr) / 2.0

        # Convert ticks → meters
        d_left_m  = d_left  * METERS_PER_TICK
        d_right_m = d_right * METERS_PER_TICK

        # ── Dead-reckoning ──
        distance = (d_left_m + d_right_m) / 2.0
        d_theta_enc = (d_right_m - d_left_m) / WHEEL_BASE

        # Time delta for IMU integration
        dt_ns = (now - self.last_enc_time).nanoseconds
        dt = dt_ns / 1e9
        if dt <= 0 or dt > 1.0:
            dt = 0.02  # fallback

        # ── Complementary filter: blend encoder heading with IMU gyro ──
        d_theta_imu = self.gyro_z * dt
        d_theta = (ENCODER_HEADING_WEIGHT * d_theta_enc +
                   (1.0 - ENCODER_HEADING_WEIGHT) * d_theta_imu)

        # ── Update pose ──
        self.theta += d_theta
        # Normalize theta to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Save for next iteration
        self.prev_enc = enc
        self.last_enc_time = now

    def _publish_odom(self):
        """Publish odometry message and TF transform at 50 Hz."""
        now = self.get_clock().now().to_msg()

        # ── Odometry message ──
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # ── TF: odom → base_link ──
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
