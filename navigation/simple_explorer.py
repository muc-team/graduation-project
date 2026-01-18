#!/usr/bin/env python3
"""
ULTRA SAFE Explorer - NEVER BUMPS
Robot: 46cm x 32cm
LiDAR at center (16cm from front, 16cm from sides)

SAFETY DISTANCES:
- Front: 60cm (very conservative)
- Sides: 50cm
- If anything closer than 30cm: REVERSE IMMEDIATELY
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

# ULTRA SAFE distances
FRONT_SAFE = 0.60      # 60cm before any obstacle
SIDE_SAFE = 0.50       # 50cm from sides
EMERGENCY_DIST = 0.30  # 30cm = STOP AND REVERSE
SPEED = 0.08           # Very slow
TURN_SPEED = 0.3


class UltraSafeExplorer(Node):
    def __init__(self):
        super().__init__('ultra_safe_explorer')
        
        self.enabled = False
        self.front = 10.0
        self.left = 10.0
        self.right = 10.0
        self.front_left = 10.0
        self.front_right = 10.0
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/explore_enable', self.enable_cb, 10)
        self.create_timer(0.2, self.navigate)
        
        self.log('Ultra Safe Explorer Started')
        self.log(f'SAFETY: Front={FRONT_SAFE}m Side={SIDE_SAFE}m Emergency={EMERGENCY_DIST}m')
    
    def log(self, msg):
        self.get_logger().info(msg)
    
    def enable_cb(self, msg):
        self.enabled = msg.data
        self.log(f'Explorer {"ENABLED" if self.enabled else "DISABLED"}')
        if not self.enabled:
            self.stop()
    
    def scan_cb(self, msg):
        """Read LiDAR - all directions"""
        r = np.array(msg.ranges)
        r = np.where(np.isfinite(r) & (r > 0.05) & (r < 12.0), r, 12.0)
        n = len(r)
        if n == 0:
            return
        
        def min_range(deg_start, deg_end):
            s = int(n * deg_start / 360) % n
            e = int(n * deg_end / 360) % n
            if s <= e:
                section = r[s:e]
            else:
                section = np.concatenate([r[s:], r[:e]])
            return float(np.min(section)) if len(section) > 0 else 12.0
        
        # FRONT: -40 to +40 degrees (wide cone)
        self.front = min(min_range(320, 360), min_range(0, 40))
        
        # FRONT-LEFT: +25 to +55
        self.front_left = min_range(25, 55)
        
        # FRONT-RIGHT: -55 to -25 (305 to 335)
        self.front_right = min_range(305, 335)
        
        # LEFT: +55 to +110
        self.left = min_range(55, 110)
        
        # RIGHT: -110 to -55 (250 to 305)
        self.right = min_range(250, 305)
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
    
    def navigate(self):
        if not self.enabled:
            return
        
        # Log every 5 seconds
        if not hasattr(self, '_cnt'):
            self._cnt = 0
        self._cnt += 1
        if self._cnt % 25 == 0:
            self.log(f'F:{self.front:.2f} FL:{self.front_left:.2f} FR:{self.front_right:.2f} L:{self.left:.2f} R:{self.right:.2f}')
        
        # EMERGENCY: Something very close!
        if self.front < EMERGENCY_DIST:
            self.log(f'🚨 EMERGENCY REVERSE! Front={self.front:.2f}m')
            self.move(-SPEED, 0)
            return
        
        if self.front_left < EMERGENCY_DIST or self.left < EMERGENCY_DIST * 0.8:
            self.log(f'🚨 Too close on LEFT! Turning right...')
            self.move(0, -TURN_SPEED)
            return
        
        if self.front_right < EMERGENCY_DIST or self.right < EMERGENCY_DIST * 0.8:
            self.log(f'🚨 Too close on RIGHT! Turning left...')
            self.move(0, TURN_SPEED)
            return
        
        # BLOCKED FRONT: Turn to clearer side
        if self.front < FRONT_SAFE:
            if self.right > self.left:
                self.move(0, -TURN_SPEED)
            else:
                self.move(0, TURN_SPEED)
            return
        
        # BLOCKED FRONT-LEFT
        if self.front_left < SIDE_SAFE:
            self.move(SPEED * 0.5, -0.15)  # Slow forward, veer right
            return
        
        # BLOCKED FRONT-RIGHT
        if self.front_right < SIDE_SAFE:
            self.move(SPEED * 0.5, 0.15)  # Slow forward, veer left
            return
        
        # ALL CLEAR - go forward
        self.move(SPEED, 0)


def main():
    rclpy.init()
    node = UltraSafeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
