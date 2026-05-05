#!/bin/bash
# ═══════════════════════════════════════════════
# ROBOT 1 (Alpha) - Optimized Startup
# ═══════════════════════════════════════════════

SESSION="robot1"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_SOURCE="source /opt/ros/humble/setup.bash"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# Grant permissions to serial ports
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null
sudo chmod 666 /dev/ttyACM0 2>/dev/null

# 1. LiDAR
tmux new-session -d -s $SESSION -n "Sensors" "$ROS_SOURCE; ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1; exec bash"

# 2. SLAM (Mapping)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; cd $DIR/../mapping && ros2 launch slam_only.py; exec bash" C-m

# 3. MOTOR BRIDGE
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; cd $DIR/../navigation && python3 robot1_bridge.py; exec bash" C-m

# 4. ROS BRIDGE
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash" C-m

# 5. EXPLORER (Autonomous)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; cd $DIR/../navigation && python3 simple_explorer.py; exec bash" C-m

tmux select-layout -t $SESSION tiled

echo "✅ Robot 1 (Alpha) Started in tmux session: $SESSION"
echo "To view logs: tmux attach -t $SESSION"
