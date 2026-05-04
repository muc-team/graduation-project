#!/bin/bash
#
# HEADLESS Raspberry Pi Startup
# Runs everything in the background for SSH sessions without gnome-terminal
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "╔═══════════════════════════════════════╗"
echo "║   RESCUE ROBOT - Headless SSH Mode    ║"
echo "╚═══════════════════════════════════════╝"

# Permissions
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null
sudo chmod 666 /dev/ttyACM0 2>/dev/null
sudo chmod 666 /dev/ttyACM1 2>/dev/null

# Function to run a ROS 2 command in the background and redirect output to a log file
run_bg() {
    local name=$1
    local cmd=$2
    echo "Starting $name... (Logs: /tmp/${name}.log)"
    bash -c "source /opt/ros/humble/setup.bash; $cmd" > "/tmp/${name}.log" 2>&1 &
}

run_bg "LiDAR" "ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1"
sleep 2

run_bg "SLAM" "cd $DIR/../mapping && ros2 launch slam_only.py"
sleep 2

run_bg "MotorController" "cd $DIR/../navigation && python3 professional_motor_controller.py"
sleep 1

run_bg "ROSBridge" "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
sleep 1

run_bg "Camera" "cd $DIR/../classification && python3 tcp_rasp.py"
sleep 1

run_bg "Explorer" "cd $DIR/../navigation && python3 simple_explorer.py"

echo ""
echo "✅ ALL SYSTEMS STARTED IN BACKGROUND!"
echo ""
echo "To view logs for any system, use:"
echo "  tail -f /tmp/MotorController.log"
echo "  tail -f /tmp/ROSBridge.log"
echo ""
echo "On PC run: python dashboard/dash_mapping.py"
