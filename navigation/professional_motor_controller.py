#!/usr/bin/env python3
"""
Professional Motor Controller for ROS 2
=========================================
Works with Arduino motor_controller_v2.ino

Features:
- Variable speed control (PWM 0-255)
- Smooth acceleration via Arduino
- Emergency stop with hard brake
- Status feedback
- Auto-reconnect
- Manual override priority
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32
import serial
import time
import threading


class ProfessionalMotorController(Node):
    def __init__(self):
        super().__init__('professional_motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)  # Higher baud for v2
        self.declare_parameter('manual_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # State
        self.arduino = None
        self.connected = False
        self.manual_mode = False
        self.manual_last_time = time.time()
        self.emergency_stop = False
        self.current_speed_pwm = 180  # PWM value 0-255
        self.last_cmd = 'S'
        
        # Connect to Arduino
        self.connect_arduino()
        
        # Subscribers
        self.auto_sub = self.create_subscription(
            Twist, '/cmd_vel', self.auto_callback, 10)
        
        self.manual_sub = self.create_subscription(
            Twist, '/manual_cmd', self.manual_callback, 10)
        
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        
        self.speed_sub = self.create_subscription(
            Float32, '/set_speed', self.speed_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.check_manual_timeout)
        self.status_timer = self.create_timer(1.0, self.request_status)
        
        self.get_logger().info('Professional Motor Controller started')
        self.get_logger().info(f'Connected: {self.connected}')
    
    def connect_arduino(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        ports = [port, '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']
        
        for p in ports:
            try:
                self.arduino = serial.Serial(p, baud, timeout=1)
                time.sleep(2)  # Wait for Arduino reset
                
                # Read startup message
                while self.arduino.in_waiting:
                    line = self.arduino.readline().decode().strip()
                    self.get_logger().info(f'Arduino: {line}')
                
                self.connected = True
                self.get_logger().info(f'✅ Connected to Arduino on {p} at {baud} baud')
                
                # Set initial speed
                self.set_speed(self.current_speed_pwm)
                return
            except Exception as e:
                self.get_logger().warn(f'Failed to connect to {p}: {e}')
        
        self.get_logger().error('❌ Could not connect to Arduino on any port')
    
    def send_command(self, cmd: str):
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(f'{cmd}\n'.encode())
                self.last_cmd = cmd
                
                # Read response
                if self.arduino.in_waiting:
                    response = self.arduino.readline().decode().strip()
                    if response:
                        self.get_logger().debug(f'Arduino response: {response}')
                
                return True
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                self.connected = False
        return False
    
    def set_speed(self, pwm_value: int):
        """Set motor speed (PWM 0-255)"""
        pwm_value = max(80, min(255, int(pwm_value)))
        self.current_speed_pwm = pwm_value
        self.send_command(f'P{pwm_value}')
    
    def twist_to_cmd(self, msg: Twist) -> str:
        """Convert Twist to Arduino command"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Dead zone
        if abs(linear) < 0.05 and abs(angular) < 0.1:
            return 'S'
        
        # Calculate speed based on magnitude
        max_linear = self.get_parameter('max_linear_speed').value
        speed_factor = abs(linear) / max_linear if max_linear > 0 else 1.0
        new_pwm = int(80 + speed_factor * 175)  # Map to 80-255
        
        if new_pwm != self.current_speed_pwm and abs(linear) > 0.05:
            self.set_speed(new_pwm)
        
        # Determine direction
        if abs(linear) > abs(angular):
            return 'F' if linear > 0 else 'B'
        else:
            return 'L' if angular > 0 else 'R'
    
    def estop_callback(self, msg: Bool):
        """Emergency stop - highest priority"""
        if msg.data:
            self.emergency_stop = True
            self.send_command('E')  # Emergency stop command
            self.get_logger().warn('🛑 EMERGENCY STOP ACTIVATED!')
            
            status = String()
            status.data = 'ESTOP'
            self.status_pub.publish(status)
        else:
            self.emergency_stop = False
            self.send_command('X')  # Release emergency
            self.get_logger().info('✅ Emergency stop released')
    
    def manual_callback(self, msg: Twist):
        """Manual command - takes priority"""
        if self.emergency_stop:
            return
        
        self.manual_mode = True
        self.manual_last_time = time.time()
        
        cmd = self.twist_to_cmd(msg)
        if cmd != self.last_cmd or cmd == 'S':
            self.send_command(cmd)
            
            status = String()
            status.data = f'MANUAL:{cmd}'
            self.status_pub.publish(status)
    
    def auto_callback(self, msg: Twist):
        """Autonomous command - only if not in manual mode"""
        if self.emergency_stop or self.manual_mode:
            return
        
        cmd = self.twist_to_cmd(msg)
        if cmd != self.last_cmd:
            self.send_command(cmd)
            
            status = String()
            status.data = f'AUTO:{cmd}'
            self.status_pub.publish(status)
    
    def speed_callback(self, msg: Float32):
        """Set speed from dashboard (0.0 - 1.0)"""
        pwm = int(80 + msg.data * 175)  # Map 0-1 to 80-255
        self.set_speed(pwm)
    
    def check_manual_timeout(self):
        """Switch back to auto mode after timeout"""
        if self.manual_mode:
            timeout = self.get_parameter('manual_timeout').value
            if time.time() - self.manual_last_time > timeout:
                self.manual_mode = False
    
    def request_status(self):
        """Request status from Arduino periodically"""
        if self.connected:
            self.send_command('?')
            
            # Read status response
            if self.arduino and self.arduino.in_waiting:
                try:
                    response = self.arduino.readline().decode().strip()
                    if response.startswith('STS:'):
                        status = String()
                        status.data = response
                        self.status_pub.publish(status)
                except:
                    pass
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.arduino:
            self.send_command('S')  # Stop
            time.sleep(0.1)
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProfessionalMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
