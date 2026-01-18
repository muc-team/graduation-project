#!/bin/bash
#
# Raspberry Pi - Lightweight Mode
# Only runs essential services (no YOLO, no heavy processing)
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     RASPBERRY PI - LIGHTWEIGHT MODE                        ║"
echo "║     Heavy processing is on PC                              ║"
echo "╚════════════════════════════════════════════════════════════╝"

# Pre-flight
echo "[CHECK] Arduino..."
if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyACM0 ]; then
    echo "✅ Arduino OK"
else
    echo "⚠️ Arduino not found"
fi

# Terminal 1: LiDAR
gnome-terminal --tab --title="LIDAR" -- bash -c "
source /opt/ros/humble/setup.bash
echo 'Starting LiDAR...'
ros2 launch rplidar_ros rplidar_a1_launch.py
exec bash"
sleep 2

# Terminal 2: SLAM
gnome-terminal --tab --title="SLAM" -- bash -c "
source /opt/ros/humble/setup.bash
echo 'Starting SLAM...'
cd $DIR/../mapping
ros2 launch start_mapping.py
exec bash"
sleep 2

# Terminal 3: Motor Controller
gnome-terminal --tab --title="MOTOR" -- bash -c "
source /opt/ros/humble/setup.bash
echo 'Starting Motor Controller...'
cd $DIR/../navigation
python3 smart_motor_controller.py
exec bash"
sleep 1

# Terminal 4: ROS Bridge
gnome-terminal --tab --title="ROS BRIDGE" -- bash -c "
source /opt/ros/humble/setup.bash
echo 'Starting ROS Bridge...'
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"
sleep 1

# Terminal 5: Camera Stream (raw, no YOLO)
gnome-terminal --tab --title="CAMERA" -- bash -c "
echo 'Starting Camera Stream (raw)...'
cd $DIR/../classification
python3 tcp_rasp.py
exec bash"

# Terminal 6: Explorer
gnome-terminal --tab --title="EXPLORER" -- bash -c "
source /opt/ros/humble/setup.bash
echo 'Explorer ready (disabled by default)'
cd $DIR/../navigation
python3 simple_explorer.py
exec bash"

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║  RPi Ready! Now run on PC:                                 ║"
echo "║                                                            ║"
echo "║  cd dashboard                                              ║"
echo "║  python dash_control.py                                    ║"
echo "║                                                            ║"
echo "║  Then open: http://localhost:8080                          ║"
echo "╚════════════════════════════════════════════════════════════╝"
