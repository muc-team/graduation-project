# Robot 2 (Beta) Architecture & Documentation

This document outlines the complete architecture, hardware configuration, serial protocols, and software bridge logic for **Robot 2** to ensure seamless maintainability in the future.

## Overview

Robot 2 uses a hybrid architecture:
- **Low-Level Controller**: Arduino Mega 2560
- **High-Level Brain**: Raspberry Pi running ROS 2 (Humble)
- **Communication Bridge**: `robot2_bridge.py`

This separation of concerns ensures that the Arduino handles hard real-time tasks like PWM signal generation, encoder interrupt handling, and I2C IMU polling, while the Pi handles networking, dashboard bridging, and EKF fusion.

---

## 1. Low-Level Firmware (Arduino)

**File location**: `arduino/robot2_controller/robot2_controller.ino`

### Hardware Wiring (Arduino Mega 2560)
- **Motor Driver (L298N)**
  - Left PWM: `Pin 10 (ENA)` | Left Dir: `Pin 8 (IN1)`, `Pin 9 (IN2)`
  - Right PWM: `Pin 11 (ENB)` | Right Dir: `Pin 12 (IN3)`, `Pin 13 (IN4)`
- **Encoders (Interrupt Pins)**
  - Front-Left (M1): `ENCA=2`, `ENCB=22`
  - Rear-Left (M2): `ENCA=3`, `ENCB=24`
  - Front-Right (M3): `ENCA=18`, `ENCB=26` (Note: Ticks negated in code due to swapped wiring)
  - Rear-Right (M4): `ENCA=19`, `ENCB=28` (Note: Ticks negated in code)
- **IMU (MPU6050 via I2C)**
  - SDA: `Pin 20` | SCL: `Pin 21`

### Arduino Execution Loop
The Arduino operates at a continuous **50 Hz stream rate** (`STREAM_HZ = 50`).
On every cycle it:
1. Checks for incoming Serial commands (F/B/L/R/S/P<pwm>).
2. Reads raw data from the MPU6050 via I2C.
3. Packages the snapshot of the 4 encoders + IMU data + timestamp.
4. Blasts the data back over Serial (`115200` baud).

---

## 2. High-Level Bridge (ROS 2 Python Node)

**File location**: `navigation/robot2_bridge.py`

### Responsibilities
The script `robot2_bridge.py` bridges the ROS 2 environment with the Arduino Mega over a USB Serial interface (`/dev/ttyUSB0` or `/dev/ttyACM0`). 

- **Subscribes to:**
  - `/cmd_vel` (geometry_msgs/Twist) - Autonomous navigation commands.
  - `/manual_cmd` (geometry_msgs/Twist) - Dashboard manual override commands.
  - `/set_speed` (std_msgs/Float32) - PWM speed scaling.
- **Publishes:**
  - `/encoders` (std_msgs/Int32MultiArray) - Array of 4 cumulative ticks.
  - `/imu/data_raw` (sensor_msgs/Imu) - Converted m/s² and rad/s data.
  - `/motor_status` (std_msgs/String) - Legacy STS formatted strings for Dashboard compatibility.

### Telemetry Conversion
The raw I2C values from the IMU are converted to physical units within `robot2_bridge.py` before being published to the ROS `/imu/data_raw` topic:
- **Accelerometers**: Multiplied by `9.81 / 8192.0` to yield `m/s²`.
- **Gyroscopes**: Multiplied by `π / (180.0 * 65.5)` to yield `rad/s`.

---

## 3. Serial Communication Protocol

The communication between the Pi and Arduino is string-based.

### Pi to Arduino (Commands)
- `F` / `B` / `L` / `R`: Move Forward, Backward, Left, Right
- `S`: Stop
- `P<0-255>`: Update the global PWM speed (e.g., `P150`)

### Arduino to Pi (Telemetry)
The 50 Hz data packet format looks like this:
`D:<timestamp>,<enc1>,<enc2>,<enc3>,<enc4>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>`

- **timestamp**: Arduino `millis()` for time delta calculations on the Pi.
- **enc1-4**: Cumulative encoder tick counts.
- **ax, ay, az**: Raw accelerometer LSBs.
- **gx, gy, gz**: Raw gyroscope LSBs.

### Legacy Dashboard Support
To maintain compatibility with older dashboard scripts that expect the format of Robot 1, the `robot2_bridge.py` automatically repacks the new `D:` telemetry string into a fake `STS:` string:
`STS:<pwm_speed>,0,<enc1>,<enc2>,<enc3>,<enc4>`
This is published to `/motor_status` to ensure dashboard GUI elements (like speed sliders and encoder readouts) function without requiring changes to the core UI logic.

---

## 4. Troubleshooting Checklist

If Robot 2 stops working in the future, follow these steps:
1. **Serial Port Check**: Ensure the Arduino is connected to the Pi and registers as `/dev/ttyUSB0` or `/dev/ttyACM0`. Check `ls /dev/tty*`.
2. **IMU Initialization**: The Arduino will print `IMU:OK` or `IMU:FAIL` on boot over serial. If it prints `FAIL`, check the I2C wiring (SDA Pin 20, SCL Pin 21) on the Mega.
3. **Encoder Ticks Not Registering**: Use `ros2 topic echo /encoders` to see if the Pi is receiving data. If not, verify the interrupt pins (`2, 3, 18, 19`).
4. **Robot Goes Backward on 'F' Command**: Swap the L298N output wires for the problematic side, or flip the polarity in `robot2_controller.ino` `drive()` logic.
5. **Node Crashes**: Relaunch the bridge manually via `ros2 run <package> robot2_bridge.py` to view raw terminal exceptions.
