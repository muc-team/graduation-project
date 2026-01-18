#!/usr/bin/env python3
"""
Motor Controller Node for Autonomous Navigation
Receives cmd_vel from Nav2 and sends commands to Arduino via USB Serial
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('linear_threshold', 0.1)
        self.declare_parameter('angular_threshold', 0.3)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Connect to Arduino
        self.arduino = None
        self.connect_arduino(port, baud)
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.last_cmd = 'S'
        self.get_logger().info(f'Motor Controller started on {port}')
    
    def connect_arduino(self, port, baud):
        """Try to connect to Arduino, with fallback ports"""
        ports_to_try = [port, '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']
        
        for p in ports_to_try:
            try:
                self.arduino = serial.Serial(p, baud, timeout=1)
                time.sleep(2)  # Wait for Arduino to reset
                self.get_logger().info(f'Connected to Arduino on {p}')
                return
            except Exception as e:
                self.get_logger().warn(f'Failed to connect to {p}: {e}')
        
        self.get_logger().error('Could not connect to Arduino on any port!')
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to Arduino command"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        linear_thresh = self.get_parameter('linear_threshold').value
        angular_thresh = self.get_parameter('angular_threshold').value
        
        # Determine command based on velocities
        if linear_x > linear_thresh:
            cmd = 'F'  # Forward
        elif linear_x < -linear_thresh:
            cmd = 'B'  # Backward
        elif angular_z > angular_thresh:
            cmd = 'L'  # Turn Left
        elif angular_z < -angular_thresh:
            cmd = 'R'  # Turn Right
        else:
            cmd = 'S'  # Stop
        
        # Only send if command changed
        if cmd != self.last_cmd:
            self.send_command(cmd)
            self.last_cmd = cmd
    
    def send_command(self, cmd: str):
        """Send command to Arduino"""
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(cmd.encode())
                self.get_logger().debug(f'Sent command: {cmd}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command: {e}')
        else:
            self.get_logger().warn('Arduino not connected')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if self.arduino:
            self.send_command('S')  # Stop motors
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
