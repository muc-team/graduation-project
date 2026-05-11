#!/usr/bin/env python3
"""
Robot 2 (Beta) — Motor & Sensor Bridge for ROS 2
=================================================
Pairs with robot2_controller.ino (Arduino Mega)

Responsibilities:
  1. Receive movement commands from dashboard → send to Arduino
  2. Parse streaming sensor data from Arduino (50 Hz)
  3. Publish /encoders  (std_msgs/Int32MultiArray)
  4. Publish /imu/data  (sensor_msgs/Imu)
  5. Publish /motor_status (std_msgs/String) for dashboard

Serial Protocol (from Arduino):
  D:timestamp,enc1,enc2,enc3,enc4,ax,ay,az,gx,gy,gz
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Imu
import serial
import time
import threading


# ═══════════════════════════════════════
# IMU Conversion Constants (must match Arduino config)
# ═══════════════════════════════════════
# Accel: AFS_SEL=1 → ±4g → 8192 LSB/g
ACCEL_SCALE = 9.81 / 8192.0    # raw → m/s²
# Gyro:  FS_SEL=1  → ±500°/s → 65.5 LSB/(°/s)
GYRO_SCALE = math.pi / (180.0 * 65.5)   # raw → rad/s


class Robot2Bridge(Node):
    def __init__(self):
        super().__init__('robot2_bridge')

        # ── Parameters ──
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('manual_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)

        # ── State ──
        self.arduino = None
        self.connected = False
        self.manual_mode = False
        self.manual_last_time = time.time()
        self.pwm_speed = 180
        self.last_cmd = 'S'

        # ── Connect to Arduino ──
        self._connect_arduino()

        # ── Subscribers ──
        self.create_subscription(Twist, '/cmd_vel', self._auto_cb, 10)
        self.create_subscription(Twist, '/manual_cmd', self._manual_cb, 10)
        self.create_subscription(Float32, '/set_speed', self._speed_cb, 10)

        # ── Publishers ──
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # ── Timers ──
        self.create_timer(0.1, self._check_manual_timeout)
        # Note: No _poll_status timer — Robot 2 firmware streams D: packets
        # at 50 Hz automatically. Sending '?' would trigger printHelp() and
        # flood the serial buffer with junk text.

        # ── Serial Reader Thread ──
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self.get_logger().info('Robot 2 Bridge started')
        self.get_logger().info(f'Arduino connected: {self.connected}')

    # ═══════════════════════════════════════
    # ARDUINO CONNECTION
    # ═══════════════════════════════════════
    def _connect_arduino(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        candidates = [port, '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']

        for p in candidates:
            try:
                self.arduino = serial.Serial(p, baud, timeout=1)
                time.sleep(2)  # Arduino resets on serial open

                # Drain startup messages
                while self.arduino.in_waiting:
                    line = self.arduino.readline().decode(errors='ignore').strip()
                    self.get_logger().info(f'Arduino: {line}')

                self.connected = True
                self.get_logger().info(f'Connected to Arduino on {p} @ {baud}')
                self._send('P{}'.format(self.pwm_speed))
                return
            except Exception as e:
                self.get_logger().warn(f'Failed {p}: {e}')

        self.get_logger().error('Could not connect to Arduino on any port')

    # ═══════════════════════════════════════
    # SERIAL COMMUNICATION
    # ═══════════════════════════════════════
    def _send(self, cmd: str):
        """Send a command to the Arduino."""
        if not (self.arduino and self.arduino.is_open):
            return False
        try:
            self.arduino.write(f'{cmd}\n'.encode())
            self.last_cmd = cmd
            return True
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.connected = False
            return False

    def _poll_status(self):
        """Periodically send '?' to request STS: data (old firmware compatibility)."""
        self._send('?')

    def _serial_reader(self):
        """Background thread: continuously reads and parses Arduino data."""
        while rclpy.ok():
            if not (self.arduino and self.arduino.is_open):
                time.sleep(1)
                continue
            try:
                raw = self.arduino.readline()
                if not raw:
                    continue
                line = raw.decode(errors='ignore').strip()
                if line.startswith('D:'):
                    self._parse_sensor_data(line)
                elif line.startswith('STS:'):
                    # Old firmware: STS:speed,estop,enc1,enc2,enc3,enc4
                    self._parse_sts_data(line)
            except Exception:
                time.sleep(0.1)

    def _parse_sts_data(self, line: str):
        """Parse old firmware format: STS:speed,estop,enc1,enc2,enc3,enc4"""
        try:
            # Publish raw status string for dashboard
            status_msg = String()
            status_msg.data = line
            self.status_pub.publish(status_msg)

            # Also publish encoder values as Int32MultiArray
            parts = line[4:].split(',')
            if len(parts) >= 6:
                encs = [int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])]
                enc_msg = Int32MultiArray()
                enc_msg.data = encs
                self.encoder_pub.publish(enc_msg)
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'STS parse error: {e}')

    def _parse_sensor_data(self, line: str):
        """Parse: D:timestamp,e1,e2,e3,e4,ax,ay,az,gx,gy,gz"""
        try:
            parts = line[2:].split(',')
            if len(parts) < 11:
                return

            ts   = int(parts[0])
            encs = [int(parts[i]) for i in range(1, 5)]
            ax, ay, az = int(parts[5]), int(parts[6]), int(parts[7])
            gx, gy, gz = int(parts[8]), int(parts[9]), int(parts[10])

            # ── Publish Encoders ──
            enc_msg = Int32MultiArray()
            enc_msg.data = encs
            self.encoder_pub.publish(enc_msg)

            # ── Publish IMU ──
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Convert raw → physical units
            imu_msg.linear_acceleration.x = ax * ACCEL_SCALE
            imu_msg.linear_acceleration.y = ay * ACCEL_SCALE
            imu_msg.linear_acceleration.z = az * ACCEL_SCALE

            imu_msg.angular_velocity.x = gx * GYRO_SCALE
            imu_msg.angular_velocity.y = gy * GYRO_SCALE
            imu_msg.angular_velocity.z = gz * GYRO_SCALE

            # Orientation unknown from raw data (EKF will compute it)
            imu_msg.orientation_covariance[0] = -1.0

            self.imu_pub.publish(imu_msg)

            # ── Publish Status (for dashboard compatibility) ──
            status_msg = String()
            status_msg.data = f'STS:{self.pwm_speed},0,{encs[0]},{encs[1]},{encs[2]},{encs[3]}'
            self.status_pub.publish(status_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error: {e}')

    # ═══════════════════════════════════════
    # MOVEMENT COMMANDS
    # ═══════════════════════════════════════
    def _twist_to_cmd(self, msg: Twist) -> str:
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < 0.05 and abs(angular) < 0.1:
            return 'S'

        # Adjust PWM based on speed magnitude
        max_lin = self.get_parameter('max_linear_speed').value
        factor = min(abs(linear) / max_lin, 1.0) if max_lin > 0 else 1.0
        new_pwm = int(80 + factor * 175)
        if new_pwm != self.pwm_speed and abs(linear) > 0.05:
            self.pwm_speed = new_pwm
            self._send(f'P{new_pwm}')

        if abs(linear) > abs(angular):
            return 'F' if linear > 0 else 'B'
        else:
            return 'L' if angular > 0 else 'R'

    def _manual_cb(self, msg: Twist):
        self.manual_mode = True
        self.manual_last_time = time.time()
        cmd = self._twist_to_cmd(msg)
        if cmd != self.last_cmd or cmd == 'S':
            self._send(cmd)

    def _auto_cb(self, msg: Twist):
        if self.manual_mode:
            return
        cmd = self._twist_to_cmd(msg)
        if cmd != self.last_cmd:
            self._send(cmd)

    def _speed_cb(self, msg: Float32):
        self.pwm_speed = int(80 + msg.data * 175)
        self._send(f'P{self.pwm_speed}')

    def _check_manual_timeout(self):
        if self.manual_mode:
            timeout = self.get_parameter('manual_timeout').value
            if time.time() - self.manual_last_time > timeout:
                self.manual_mode = False

    # ═══════════════════════════════════════
    # SHUTDOWN
    # ═══════════════════════════════════════
    def destroy_node(self):
        if self.arduino:
            self._send('S')
            time.sleep(0.1)
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
