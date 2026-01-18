#!/bin/bash
# Autonomous Navigation Startup Script
# Run this on Raspberry Pi to start the full autonomous navigation system

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "========================================"
echo "   AUTONOMOUS NAVIGATION STARTUP"
echo "========================================"

# Check if Arduino is connected
if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyACM0 ]; then
    echo "✅ Arduino detected"
else
    echo "⚠️  Warning: Arduino not detected on USB"
    echo "   Please connect Arduino and try again"
fi

# Terminal 1: Autonomous Navigation (SLAM + Nav2 + Motor Control)
gnome-terminal --tab --title="AUTONOMOUS NAV" -- bash -c "
source /opt/ros/humble/setup.bash;
echo '🚀 Starting Autonomous Navigation...';
ros2 launch ../navigation/start_autonomous.py;
exec bash"

# Terminal 2: ROS Bridge for Dashboard
gnome-terminal --tab --title="ROS BRIDGE" -- bash -c "
source /opt/ros/humble/setup.bash;
echo '🌐 Starting ROS Bridge...';
ros2 launch rosbridge_server rosbridge_websocket_launch.xml;
exec bash"

# Terminal 3: Camera Stream
gnome-terminal --tab --title="CAMERA" -- bash -c "
echo '📷 Starting Camera Stream...';
if [ -f ../classification/tcp_rasp.py ]; then
    python3 ../classification/tcp_rasp.py;
else
    echo 'Error: tcp_rasp.py not found!';
fi;
exec bash"

# Terminal 4: Manual Command Interface
gnome-terminal --tab --title="MANUAL CONTROL" -- bash -c "
source /opt/ros/humble/setup.bash;
echo '========================================';
echo '  MANUAL CONTROL INTERFACE';
echo '========================================';
echo '';
echo 'Commands:';
echo '  Forward:  ros2 topic pub /cmd_vel geometry_msgs/Twist \"{linear: {x: 0.2}}\" -1';
echo '  Backward: ros2 topic pub /cmd_vel geometry_msgs/Twist \"{linear: {x: -0.2}}\" -1';
echo '  Left:     ros2 topic pub /cmd_vel geometry_msgs/Twist \"{angular: {z: 0.5}}\" -1';
echo '  Right:    ros2 topic pub /cmd_vel geometry_msgs/Twist \"{angular: {z: -0.5}}\" -1';
echo '  Stop:     ros2 topic pub /cmd_vel geometry_msgs/Twist \"{}\" -1';
echo '';
echo 'In RViz: Use \"2D Goal Pose\" to send navigation goals';
echo '';
exec bash"

echo ""
echo "✅ All systems starting..."
echo "   - Autonomous Navigation"
echo "   - ROS Bridge (for Dashboard)"
echo "   - Camera Stream"
echo "   - Manual Control Interface"
echo ""
echo "🎯 Use RViz '2D Goal Pose' button to navigate!"
