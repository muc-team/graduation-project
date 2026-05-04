#!/usr/bin/env python3
"""
Robot 1 (Alpha) — Motor Bridge for ROS 2
=========================================
Pairs with the standard motor controller on Robot 1.
Handles the STS: protocol for basic status and encoders.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
import serial
import time
import threading


class Robot1Bridge(Node):
    def __init__(self):
        super().__init__('robot1_bridge')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.arduino = None
        self.connect_arduino()
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        
        self.last_cmd = 'S'
        
        # Status polling thread
        self.thread = threading.Thread(target=self.poll_status, daemon=True)
        self.thread.start()
        
        self.get_logger().info('Robot 1 Bridge started')

    def connect_arduino(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        ports = [port, '/dev/ttyACM0', '/dev/ttyUSB1']
        
        for p in ports:
            try:
                self.arduino = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'Connected to Arduino on {p}')
                return
            except:
                pass
        self.get_logger().error('Could not connect to Robot 1 Arduino')

    def cmd_vel_callback(self, msg: Twist):
        if not self.arduino: return
        
        linear = msg.linear.x
        angular = msg.angular.z
        
        if linear > 0.1: cmd = 'F'
        elif linear < -0.1: cmd = 'B'
        elif angular > 0.3: cmd = 'L'
        elif angular < -0.3: cmd = 'R'
        else: cmd = 'S'
        
        if cmd != self.last_cmd:
            self.arduino.write(f'{cmd}\n'.encode())
            self.last_cmd = cmd

    def poll_status(self):
        while rclpy.ok():
            if self.arduino:
                try:
                    self.arduino.write(b'?\n') # Request status
                    line = self.arduino.readline().decode().strip()
                    if line.startswith('STS:'):
                        # Publish raw status for dashboard
                        self.status_pub.publish(String(data=line))
                        
                        # Parse encoders for ROS
                        parts = line.split(',')
                        if len(parts) >= 6:
                            enc_msg = Int32MultiArray()
                            enc_msg.data = [int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])]
                            self.encoder_pub.publish(enc_msg)
                except:
                    pass
            time.sleep(0.2) # 5Hz polling

def main(args=None):
    rclpy.init(args=args)
    node = Robot1Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
