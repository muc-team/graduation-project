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
from std_msgs.msg import String, Int32MultiArray, Bool
import serial
import time
import threading


class Robot1Bridge(Node):
    def __init__(self):
        super().__init__('robot1_bridge')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('manual_timeout', 0.5)
        
        self.arduino = None
        self.connect_arduino()
        
        # Manual override state
        self.manual_mode = False
        self.manual_last_time = time.time()
        self.emergency_stop = False
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/manual_cmd', self.manual_cmd_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        
        self.last_cmd = 'S'
        
        # Timers
        self.create_timer(0.1, self.check_manual_timeout)
        
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

    def _twist_to_cmd(self, msg: Twist) -> str:
        """Convert a Twist message to an Arduino direction command."""
        linear = msg.linear.x
        angular = msg.angular.z
        if abs(linear) < 0.05 and abs(angular) < 0.1:
            return 'S'
        if abs(linear) > abs(angular):
            return 'F' if linear > 0 else 'B'
        else:
            return 'L' if angular > 0 else 'R'

    def _send(self, cmd: str):
        """Send a command to the Arduino if it differs from the last one."""
        if not self.arduino:
            return
        if cmd != self.last_cmd or cmd == 'S':
            try:
                self.arduino.write(f'{cmd}\n'.encode())
                self.last_cmd = cmd
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Autonomous command — only processed when not in manual mode."""
        if self.emergency_stop or self.manual_mode:
            return
        self._send(self._twist_to_cmd(msg))

    def manual_cmd_callback(self, msg: Twist):
        """Manual command from dashboard — takes priority over autonomous."""
        if self.emergency_stop:
            return
        self.manual_mode = True
        self.manual_last_time = time.time()
        self._send(self._twist_to_cmd(msg))

    def estop_callback(self, msg: Bool):
        """Emergency stop — highest priority."""
        if msg.data:
            self.emergency_stop = True
            self._send('S')
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        else:
            self.emergency_stop = False
            self.get_logger().info('Emergency stop released')

    def check_manual_timeout(self):
        """Return to autonomous mode after manual timeout."""
        if self.manual_mode:
            timeout = self.get_parameter('manual_timeout').value
            if time.time() - self.manual_last_time > timeout:
                self.manual_mode = False

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
