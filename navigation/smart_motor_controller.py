#!/usr/bin/env python3
"""
Smart Motor Controller with Manual Override
- Listens to /cmd_vel for autonomous navigation
- Manual commands from /manual_cmd have PRIORITY
- Emergency stop always works
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import serial
import time
import threading


class SmartMotorController(Node):
    def __init__(self):
        super().__init__('smart_motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('manual_timeout', 0.5)  # Manual override lasts 0.5s
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Serial connection
        self.arduino = None
        self.connect_arduino(port, baud)
        
        # State
        self.last_cmd = 'S'
        self.manual_mode = False
        self.manual_last_time = time.time()
        self.emergency_stop = False
        
        # Subscribers
        self.auto_sub = self.create_subscription(
            Twist, '/cmd_vel', self.auto_callback, 10)
        
        self.manual_sub = self.create_subscription(
            Twist, '/manual_cmd', self.manual_callback, 10)
        
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        
        # Timer to check manual timeout
        self.timer = self.create_timer(0.1, self.check_manual_timeout)
        
        self.get_logger().info(f'Smart Motor Controller started on {port}')
        self.get_logger().info('Manual override: publish to /manual_cmd')
        self.get_logger().info('Emergency stop: publish to /emergency_stop')
    
    def connect_arduino(self, port, baud):
        ports = [port, '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyAMA0']
        for p in ports:
            try:
                self.arduino = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'Connected to Arduino on {p}')
                return
            except:
                pass
        self.get_logger().error('Could not connect to Arduino!')
    
    def twist_to_cmd(self, msg):
        """Convert Twist to Arduino command"""
        if abs(msg.linear.x) > 0.05:
            return 'F' if msg.linear.x > 0 else 'B'
        elif abs(msg.angular.z) > 0.1:
            return 'L' if msg.angular.z > 0 else 'R'
        return 'S'
    
    def send_command(self, cmd):
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(cmd.encode())
                self.last_cmd = cmd
                
                # Publish status
                status = String()
                mode = 'MANUAL' if self.manual_mode else 'AUTO'
                status.data = f'{mode}:{cmd}'
                self.status_pub.publish(status)
            except:
                pass
    
    def estop_callback(self, msg):
        """Emergency stop - highest priority"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.send_command('S')
            self.get_logger().warn('🛑 EMERGENCY STOP ACTIVATED!')
    
    def manual_callback(self, msg):
        """Manual command - takes priority over auto"""
        if self.emergency_stop:
            return
        
        self.manual_mode = True
        self.manual_last_time = time.time()
        cmd = self.twist_to_cmd(msg)
        
        if cmd != self.last_cmd:
            self.send_command(cmd)
            self.get_logger().info(f'[MANUAL] {cmd}')
    
    def auto_callback(self, msg):
        """Autonomous command - only if not in manual mode"""
        if self.emergency_stop:
            return
        
        if self.manual_mode:
            return  # Manual has priority
        
        cmd = self.twist_to_cmd(msg)
        if cmd != self.last_cmd:
            self.send_command(cmd)
    
    def check_manual_timeout(self):
        """Reset to auto mode after manual timeout"""
        if self.manual_mode:
            timeout = self.get_parameter('manual_timeout').value
            if time.time() - self.manual_last_time > timeout:
                self.manual_mode = False
                self.get_logger().info('Switched back to AUTO mode')
    
    def destroy_node(self):
        if self.arduino:
            self.send_command('S')
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmartMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
