#!/usr/bin/env python3
"""
Fake Odometry Node
Estimates robot position from cmd_vel (for LiDAR-only navigation)
This is a simple dead-reckoning approach - SLAM scan matching will correct drift
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


class FakeOdometry(Node):
    def __init__(self):
        super().__init__('fake_odom')
        
        # Parameters
        self.declare_parameter('linear_scale', 0.2)   # m/s when moving
        self.declare_parameter('angular_scale', 0.5)  # rad/s when turning
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Timer for publishing
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_odom)
        
        self.get_logger().info('Fake Odometry node started')
    
    def cmd_vel_callback(self, msg: Twist):
        """Update velocities from cmd_vel"""
        linear_scale = self.get_parameter('linear_scale').value
        angular_scale = self.get_parameter('angular_scale').value
        
        # Scale the commanded velocities
        self.vx = msg.linear.x * linear_scale if abs(msg.linear.x) > 0.01 else 0.0
        self.vth = msg.angular.z * angular_scale if abs(msg.angular.z) > 0.01 else 0.0
    
    def publish_odom(self):
        """Compute and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Compute position change
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from theta
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
