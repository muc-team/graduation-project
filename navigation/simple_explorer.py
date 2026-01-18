#!/usr/bin/env python3
"""
Simple Frontier Exploration
Makes the robot explore unknown areas automatically
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import random
import math


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter('obstacle_distance', 0.5)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.enabled_sub = self.create_subscription(
            Bool, '/explore_enable', self.enable_callback, 10)
        
        # State
        self.enabled = False
        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.turn_direction = 1
        self.stuck_count = 0
        
        # Timer for exploration loop
        self.timer = self.create_timer(0.2, self.explore_loop)
        
        self.get_logger().info('Simple Explorer ready')
        self.get_logger().info('Publish to /explore_enable to start/stop')
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info('🚀 Exploration ENABLED')
        else:
            self.get_logger().info('⏹️ Exploration DISABLED')
            self.stop()
    
    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def move(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
    
    def check_obstacle_ahead(self):
        """Check if there's an obstacle ahead using the map"""
        if self.map_data is None:
            return False
        
        # Simple check: look at cells in front of robot (center of map)
        h, w = self.map_data.shape
        center_y, center_x = h // 2, w // 2
        
        # Check a small region ahead
        look_ahead = 10  # cells
        region = self.map_data[center_y:center_y+look_ahead, center_x-5:center_x+5]
        
        # If any cell is occupied (100) or unknown (-1 stored as 255 due to uint8)
        if np.any(region == 100):
            return True
        return False
    
    def find_frontier(self):
        """Find direction to nearest frontier (boundary between known and unknown)"""
        if self.map_data is None:
            return None
        
        h, w = self.map_data.shape
        center_y, center_x = h // 2, w // 2
        
        # Look for unknown areas (-1) adjacent to free areas (0)
        frontiers = []
        
        for y in range(1, h-1):
            for x in range(1, w-1):
                if self.map_data[y, x] == 0:  # Free cell
                    # Check neighbors for unknown
                    neighbors = [
                        self.map_data[y-1, x], self.map_data[y+1, x],
                        self.map_data[y, x-1], self.map_data[y, x+1]
                    ]
                    if -1 in neighbors:
                        frontiers.append((x - center_x, y - center_y))
        
        if not frontiers:
            return None
        
        # Return direction to random frontier
        fx, fy = random.choice(frontiers)
        return math.atan2(fy, fx)
    
    def explore_loop(self):
        """Main exploration logic"""
        if not self.enabled:
            return
        
        linear = self.get_parameter('linear_speed').value
        angular = self.get_parameter('angular_speed').value
        
        # Simple behavior: move forward, turn when obstacle
        if self.check_obstacle_ahead():
            # Obstacle! Turn
            self.stuck_count += 1
            
            if self.stuck_count > 15:
                # Really stuck, reverse
                self.move(-linear, 0)
                self.stuck_count = 0
            else:
                # Turn in consistent direction
                self.move(0, angular * self.turn_direction)
                
                # Occasionally change turn direction
                if random.random() < 0.1:
                    self.turn_direction *= -1
        else:
            # Clear ahead, move forward
            self.stuck_count = 0
            self.move(linear, 0)
    
    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
