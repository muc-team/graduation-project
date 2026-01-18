#!/bin/bash
#
# MINIMAL Raspberry Pi Startup
# No RViz, No extra load
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "╔═══════════════════════════════════════╗"
echo "║     RESCUE ROBOT - Minimal Mode       ║"
echo "╚═══════════════════════════════════════╝"

# Permissions
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null

echo "[1/5] Starting LiDAR..."
gnome-terminal --tab --title="LiDAR" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
exec bash" &
sleep 4

echo "[2/5] Starting SLAM (no RViz)..."
gnome-terminal --tab --title="SLAM" -- bash -c "
source /opt/ros/humble/setup.bash
cd $DIR/../mapping
ros2 launch slam_only.py
exec bash" &
sleep 2

echo "[3/5] Starting Motor Controller..."
gnome-terminal --tab --title="Motor" -- bash -c "
source /opt/ros/humble/setup.bash
cd $DIR/../navigation
python3 professional_motor_controller.py
exec bash" &
sleep 1

echo "[4/5] Starting ROS Bridge..."
gnome-terminal --tab --title="Bridge" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash" &
sleep 1

echo "[5/5] Starting Camera..."
gnome-terminal --tab --title="Camera" -- bash -c "
cd $DIR/../classification
python3 tcp_rasp.py 2>/dev/null || echo 'Camera not available'
exec bash" &

echo ""
echo "✅ READY!"
echo ""
echo "On PC run:"
echo "  python professional_dashboard.py"
echo ""
echo "Open: http://localhost:8080"
