#!/usr/bin/env python3
"""
Ultra Smart Explorer - Never bumps into walls
Uses LiDAR for 360° obstacle detection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

# Robot: 46cm x 32cm, LiDAR center
SAFE_FRONT = 0.50   # 50cm front clearance
SAFE_SIDE = 0.40    # 40cm side clearance
SAFE_BACK = 0.30    # 30cm back clearance
DANGER_ZONE = 0.25  # 25cm = too close!


class UltraExplorer(Node):
    def __init__(self):
        super().__init__('ultra_explorer')
        
        self.speed = 0.10       # Slow for safety
        self.turn_speed = 0.35
        self.enabled = False
        
        # Distances
        self.front = 10.0
        self.front_left = 10.0
        self.front_right = 10.0
        self.left = 10.0
        self.right = 10.0
        self.back = 10.0
        
        # Stuck detection
        self.stuck_count = 0
        self.last_action = None
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/explore_enable', self.enable_cb, 10)
        self.create_timer(0.15, self.think)
        
        self.get_logger().info('🤖 Ultra Explorer Ready')
        self.get_logger().info(f'Safety: Front={SAFE_FRONT}m Side={SAFE_SIDE}m')
    
    def enable_cb(self, msg):
        self.enabled = msg.data
        self.stuck_count = 0
        if self.enabled:
            self.get_logger().info('✅ ENABLED')
        else:
            self.get_logger().info('⏹️ DISABLED')
            self.stop()
    
    def scan_cb(self, msg):
        """Process 360° LiDAR scan"""
        r = np.array(msg.ranges)
        r = np.where(np.isfinite(r) & (r > 0.1), r, 10.0)
        n = len(r)
        
        def get_min(start_deg, end_deg):
            s = int(n * start_deg / 360)
            e = int(n * end_deg / 360)
            if s < e:
                return float(np.min(r[s:e]))
            else:
                return float(min(np.min(r[s:]), np.min(r[:e])))
        
        # Front: -45° to +45°
        self.front = get_min(315, 45)
        # Front-Left: +30° to +60°
        self.front_left = get_min(30, 60)
        # Front-Right: -60° to -30° (300° to 330°)
        self.front_right = get_min(300, 330)
        # Left: +60° to +120°
        self.left = get_min(60, 120)
        # Right: +240° to +300°
        self.right = get_min(240, 300)
        # Back: +135° to +225°
        self.back = get_min(135, 225)
    
    def move(self, lin, ang, action=""):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)
        
        if action != self.last_action:
            self.get_logger().info(f'{action} | F:{self.front:.2f} L:{self.left:.2f} R:{self.right:.2f}')
            self.last_action = action
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def think(self):
        """Main decision logic"""
        if not self.enabled:
            return
        
        # DANGER: Too close! Reverse!
        if self.front < DANGER_ZONE:
            self.get_logger().warn(f'⚠️ DANGER! Front={self.front:.2f}m - REVERSING')
            self.move(-self.speed, 0, "🔙 REVERSE")
            self.stuck_count += 1
            return
        
        # Check if stuck (same action too many times)
        if self.stuck_count > 20:
            self.get_logger().warn('🔄 Stuck! Random turn...')
            import random
            self.move(0, random.choice([-1, 1]) * self.turn_speed * 2, "🔄 UNSTICK")
            self.stuck_count = 0
            return
        
        # FRONT BLOCKED
        if self.front < SAFE_FRONT:
            self.stuck_count += 1
            
            # Decide turn direction
            if self.right > self.left and self.right > SAFE_SIDE:
                self.move(0, -self.turn_speed, "↪️ Turn RIGHT")
            elif self.left > SAFE_SIDE:
                self.move(0, self.turn_speed, "↩️ Turn LEFT")
            elif self.right > self.left:
                self.move(0, -self.turn_speed, "↪️ Force RIGHT")
            else:
                self.move(0, self.turn_speed, "↩️ Force LEFT")
            return
        
        # SIDES TOO CLOSE - Adjust
        if self.left < SAFE_SIDE * 0.7:
            self.move(self.speed * 0.5, -0.2, "➡️ Veer right")
            return
        
        if self.right < SAFE_SIDE * 0.7:
            self.move(self.speed * 0.5, 0.2, "⬅️ Veer left")
            return
        
        # CLEAR - Go forward
        self.stuck_count = 0
        
        # Slight curve towards more open side
        curve = 0.0
        if self.front_left > self.front_right + 0.3:
            curve = 0.1  # Curve left
        elif self.front_right > self.front_left + 0.3:
            curve = -0.1  # Curve right
        
        self.move(self.speed, curve, "⬆️ FORWARD")


def main():
    rclpy.init()
    node = UltraExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
