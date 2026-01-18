#!/usr/bin/env python3
"""
Emergency Keyboard Controller
Press SPACE or ESC to STOP the robot immediately!
Press arrow keys for manual control
Press Q to quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class EmergencyController(Node):
    def __init__(self):
        super().__init__('emergency_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('=' * 50)
        self.get_logger().info('EMERGENCY KEYBOARD CONTROLLER')
        self.get_logger().info('=' * 50)
        self.get_logger().info('SPACE / ESC = EMERGENCY STOP')
        self.get_logger().info('Arrow Keys  = Manual Control')
        self.get_logger().info('S           = Stop')
        self.get_logger().info('Q           = Quit')
        self.get_logger().info('=' * 50)
    
    def send_cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        if linear == 0 and angular == 0:
            self.get_logger().warn('🛑 STOPPED!')
        else:
            self.get_logger().info(f'Moving: linear={linear:.1f}, angular={angular:.1f}')
    
    def emergency_stop(self):
        """Send multiple stop commands to ensure it stops"""
        for _ in range(5):
            self.send_cmd(0.0, 0.0)
        self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED! 🚨')


def get_key():
    """Get a single keypress"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            return ch + ch2
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = EmergencyController()
    
    LINEAR_SPEED = 0.2
    ANGULAR_SPEED = 0.5
    
    try:
        while True:
            key = get_key()
            
            # Emergency Stop
            if key == ' ' or key == '\x1b':  # Space or ESC
                node.emergency_stop()
            
            # Arrow keys
            elif key == '\x1b[A':  # Up
                node.send_cmd(LINEAR_SPEED, 0.0)
            elif key == '\x1b[B':  # Down
                node.send_cmd(-LINEAR_SPEED, 0.0)
            elif key == '\x1b[C':  # Right
                node.send_cmd(0.0, -ANGULAR_SPEED)
            elif key == '\x1b[D':  # Left
                node.send_cmd(0.0, ANGULAR_SPEED)
            
            # Other keys
            elif key.lower() == 's':
                node.send_cmd(0.0, 0.0)
            elif key.lower() == 'w':
                node.send_cmd(LINEAR_SPEED, 0.0)
            elif key.lower() == 'a':
                node.send_cmd(0.0, ANGULAR_SPEED)
            elif key.lower() == 'd':
                node.send_cmd(0.0, -ANGULAR_SPEED)
            elif key.lower() == 'x':
                node.send_cmd(-LINEAR_SPEED, 0.0)
            elif key.lower() == 'q':
                node.emergency_stop()
                break
            
    except Exception as e:
        node.emergency_stop()
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\nController stopped.")


if __name__ == '__main__':
    main()
