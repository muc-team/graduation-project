#!/usr/bin/env python3
"""
Smart Explorer with Robot Dimensions
Robot: 46cm x 32cm
LiDAR: Center (16cm from front, 16cm from sides)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math

# Robot Dimensions (meters)
ROBOT_LENGTH = 0.46  # 46cm
ROBOT_WIDTH = 0.32   # 32cm
LIDAR_OFFSET_FRONT = 0.16  # 16cm from front
SAFETY_MARGIN = 0.15  # 15cm extra safety

# Minimum distances
MIN_FRONT = LIDAR_OFFSET_FRONT + SAFETY_MARGIN  # 31cm
MIN_SIDE = (ROBOT_WIDTH / 2) + SAFETY_MARGIN     # 31cm


class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        
        # Parameters
        self.linear_speed = 0.12  # Slower for safety
        self.angular_speed = 0.4
        
        # State
        self.enabled = False
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.min_front_dist = 10.0
        self.min_left_dist = 10.0
        self.min_right_dist = 10.0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.enable_sub = self.create_subscription(Bool, '/explore_enable', self.enable_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.2, self.explore_loop)
        
        self.get_logger().info('Smart Explorer Ready')
        self.get_logger().info(f'Robot: {ROBOT_LENGTH*100}cm x {ROBOT_WIDTH*100}cm')
        self.get_logger().info(f'Safety distances - Front: {MIN_FRONT*100}cm, Sides: {MIN_SIDE*100}cm')
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info('🚀 Exploration ENABLED')
        else:
            self.get_logger().info('⏹️ Exploration DISABLED')
            self.stop()
    
    def scan_callback(self, msg):
        """Process LiDAR scan - check all directions"""
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        n = len(ranges)
        
        # Front: -30° to +30° (center)
        front_start = int(n * 330 / 360)  # -30°
        front_end = int(n * 30 / 360)     # +30°
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])
        self.min_front_dist = np.min(front_ranges) if len(front_ranges) > 0 else 10.0
        
        # Front-Left: +30° to +60°
        fl_start = int(n * 30 / 360)
        fl_end = int(n * 60 / 360)
        fl_ranges = ranges[fl_start:fl_end]
        
        # Front-Right: -60° to -30° (300° to 330°)
        fr_start = int(n * 300 / 360)
        fr_end = int(n * 330 / 360)
        fr_ranges = ranges[fr_start:fr_end]
        
        # Left: +60° to +120°
        left_start = int(n * 60 / 360)
        left_end = int(n * 120 / 360)
        left_ranges = ranges[left_start:left_end]
        self.min_left_dist = np.min(left_ranges) if len(left_ranges) > 0 else 10.0
        
        # Right: -120° to -60° (240° to 300°)
        right_start = int(n * 240 / 360)
        right_end = int(n * 300 / 360)
        right_ranges = ranges[right_start:right_end]
        self.min_right_dist = np.min(right_ranges) if len(right_ranges) > 0 else 10.0
        
        # Check clearances
        self.front_clear = self.min_front_dist > MIN_FRONT
        self.left_clear = self.min_left_dist > MIN_SIDE
        self.right_clear = self.min_right_dist > MIN_SIDE
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def move(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
    
    def explore_loop(self):
        if not self.enabled:
            return
        
        # Log distances periodically
        if hasattr(self, '_log_count'):
            self._log_count += 1
        else:
            self._log_count = 0
        
        if self._log_count % 25 == 0:  # Every 5 seconds
            self.get_logger().info(f'Dist: F={self.min_front_dist:.2f}m L={self.min_left_dist:.2f}m R={self.min_right_dist:.2f}m')
        
        # Decision making
        if self.min_front_dist < 0.20:  # Very close! Reverse
            self.get_logger().warn('⚠️ Too close! Reversing...')
            self.move(-self.linear_speed, 0.0)
        
        elif not self.front_clear:
            # Obstacle ahead - turn
            if self.right_clear and not self.left_clear:
                self.move(0.0, -self.angular_speed)  # Turn right
            elif self.left_clear and not self.right_clear:
                self.move(0.0, self.angular_speed)   # Turn left
            elif self.right_clear:
                self.move(0.0, -self.angular_speed)  # Default: turn right
            elif self.left_clear:
                self.move(0.0, self.angular_speed)   # Turn left
            else:
                # Stuck! Reverse and turn
                self.move(-self.linear_speed, self.angular_speed)
        
        else:
            # Front clear - move forward
            # Adjust slightly away from close walls
            angular = 0.0
            if self.min_left_dist < MIN_SIDE * 1.5:
                angular = -0.1  # Veer right
            elif self.min_right_dist < MIN_SIDE * 1.5:
                angular = 0.1   # Veer left
            
            self.move(self.linear_speed, angular)
    
    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmartExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
