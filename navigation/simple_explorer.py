#!/usr/bin/env python3
"""
SMART Corridor Explorer - Map-Aware Navigation
Robot: 46cm x 32cm

Features:
- Corridor detection with wall-following
- Turn persistence to prevent oscillation
- State machine for consistent behavior
- Reduced distances for narrow spaces
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import time

# Safer distances - prevents wall crashes while allowing corridor navigation
FRONT_SAFE = 0.50       # 50cm before obstacle (balance between safe and workable)
SIDE_SAFE = 0.40        # 40cm from sides
EMERGENCY_DIST = 0.30   # 30cm = STOP AND REVERSE
CORRIDOR_WIDTH = 0.65   # Corridor detected if both sides < 65cm

SPEED = 0.06            # Slower forward speed for safety
TURN_SPEED = 0.30       # Turn speed
CORRIDOR_SPEED = 0.05   # Very slow in corridors

# State machine
STATE_FORWARD = 0
STATE_CORRIDOR = 1
STATE_TURNING = 2
STATE_REVERSING = 3


class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        
        self.enabled = False
        self.state = STATE_FORWARD
        
        # LiDAR readings
        self.front = 10.0
        self.left = 10.0
        self.right = 10.0
        self.front_left = 10.0
        self.front_right = 10.0
        
        # Turn persistence - prevents oscillation
        self.turn_direction = 0  # -1 = right, 0 = none, 1 = left
        self.turn_start_time = 0
        self.turn_lock_duration = 1.2  # Lock turn for 1.2 seconds
        
        # History for smarter decisions
        self.left_history = []
        self.right_history = []
        self.history_size = 5
        
        # Stuck detection
        self.last_positions = []
        self.stuck_counter = 0
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/explore_enable', self.enable_cb, 10)
        self.create_timer(0.15, self.navigate)  # Slightly faster updates
        
        self.log('🚀 Smart Corridor Explorer Started')
        self.log(f'Distances: Front={FRONT_SAFE}m Side={SIDE_SAFE}m Emergency={EMERGENCY_DIST}m')
    
    def log(self, msg):
        self.get_logger().info(msg)
    
    def enable_cb(self, msg):
        self.enabled = msg.data
        self.log(f'Explorer {"ENABLED" if self.enabled else "DISABLED"}')
        if not self.enabled:
            self.stop()
            self.state = STATE_FORWARD
            self.turn_direction = 0
    
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
        
        # FRONT: -35 to +35 degrees (slightly narrower cone)
        self.front = min(min_range(325, 360), min_range(0, 35))
        
        # FRONT-LEFT: +20 to +50
        self.front_left = min_range(20, 50)
        
        # FRONT-RIGHT: -50 to -20 (310 to 340)
        self.front_right = min_range(310, 340)
        
        # LEFT: +50 to +100
        self.left = min_range(50, 100)
        
        # RIGHT: -100 to -50 (260 to 310)
        self.right = min_range(260, 310)
        
        # Update history for smarter decisions
        self.left_history.append(self.left)
        self.right_history.append(self.right)
        if len(self.left_history) > self.history_size:
            self.left_history.pop(0)
            self.right_history.pop(0)
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
    
    def is_turn_locked(self):
        """Check if we're locked into a turn direction"""
        if self.turn_direction == 0:
            return False
        return (time.time() - self.turn_start_time) < self.turn_lock_duration
    
    def set_turn(self, direction):
        """Set turn direction with lock (-1=right, 1=left)"""
        if not self.is_turn_locked():
            self.turn_direction = direction
            self.turn_start_time = time.time()
    
    def get_avg_space(self, history):
        """Get average space from history"""
        if not history:
            return 10.0
        return sum(history) / len(history)
    
    def is_in_corridor(self):
        """Detect if robot is in a narrow corridor"""
        return self.left < CORRIDOR_WIDTH and self.right < CORRIDOR_WIDTH
    
    def navigate(self):
        if not self.enabled:
            return
        
        # Debug logging every 3 seconds
        if not hasattr(self, '_cnt'):
            self._cnt = 0
        self._cnt += 1
        if self._cnt % 20 == 0:
            state_name = ['FORWARD', 'CORRIDOR', 'TURNING', 'REVERSING'][self.state]
            self.log(f'[{state_name}] F:{self.front:.2f} L:{self.left:.2f} R:{self.right:.2f} Turn:{self.turn_direction}')
        
        # ============ EMERGENCY HANDLING ============
        if self.front < EMERGENCY_DIST:
            self.state = STATE_REVERSING
            self.log(f'🚨 EMERGENCY! Front={self.front:.2f}m - Reversing')
            self.move(-SPEED, 0)
            return
        
        if self.front_left < EMERGENCY_DIST or self.left < EMERGENCY_DIST * 0.8:
            self.log(f'🚨 Left emergency! Veering right')
            self.set_turn(-1)
            self.move(0, -TURN_SPEED * 0.8)
            return
        
        if self.front_right < EMERGENCY_DIST or self.right < EMERGENCY_DIST * 0.8:
            self.log(f'🚨 Right emergency! Veering left')
            self.set_turn(1)
            self.move(0, TURN_SPEED * 0.8)
            return
        
        # ============ CORRIDOR MODE ============
        if self.is_in_corridor():
            self.state = STATE_CORRIDOR
            
            # In corridor: center between walls
            diff = self.left - self.right
            correction = np.clip(diff * 0.3, -0.2, 0.2)
            
            if self.front < FRONT_SAFE:
                # Blocked in corridor - must turn
                if self.is_turn_locked():
                    self.move(0, self.turn_direction * TURN_SPEED)
                else:
                    # Choose based on history, not instant reading
                    avg_left = self.get_avg_space(self.left_history)
                    avg_right = self.get_avg_space(self.right_history)
                    
                    if avg_right > avg_left + 0.05:  # Need 5cm difference to switch
                        self.set_turn(-1)
                    else:
                        self.set_turn(1)
                    self.move(0, self.turn_direction * TURN_SPEED)
            else:
                # Corridor clear ahead - move with centering
                self.move(CORRIDOR_SPEED, correction)
            return
        
        # ============ NORMAL NAVIGATION ============
        self.state = STATE_FORWARD
        
        # Front blocked - need to turn
        if self.front < FRONT_SAFE:
            self.state = STATE_TURNING
            
            if self.is_turn_locked():
                # Continue previous turn direction
                self.move(0, self.turn_direction * TURN_SPEED)
            else:
                # Choose new direction based on average space
                avg_left = self.get_avg_space(self.left_history)
                avg_right = self.get_avg_space(self.right_history)
                
                if avg_right > avg_left + 0.10:  # 10cm hysteresis
                    self.set_turn(-1)
                    self.log(f'↻ Turning RIGHT (L:{avg_left:.2f} R:{avg_right:.2f})')
                else:
                    self.set_turn(1)
                    self.log(f'↺ Turning LEFT (L:{avg_left:.2f} R:{avg_right:.2f})')
                
                self.move(0, self.turn_direction * TURN_SPEED)
            return
        
        # Gentle corrections near walls
        if self.front_left < SIDE_SAFE:
            self.move(SPEED * 0.7, -0.12)
            return
        
        if self.front_right < SIDE_SAFE:
            self.move(SPEED * 0.7, 0.12)
            return
        
        # All clear - full speed ahead, release turn lock
        self.turn_direction = 0
        self.move(SPEED, 0)


def main():
    rclpy.init()
    node = SmartExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
