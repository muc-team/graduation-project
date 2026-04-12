#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

SESSION="dash_system"

# Local workspace source path (Crucial for ROS 2)
# Make sure this path points to your actual install folder
WS_SOURCE="source ~/rplidar_project/ros2_ws/install/setup.bash"
ROS_SOURCE="source /opt/ros/humble/setup.bash"

tmux kill-session -t $SESSION 2>/dev/null

# 1. MAPPING SYSTEM
tmux new-session -d -s $SESSION -n "Main" "$ROS_SOURCE; $WS_SOURCE; ros2 launch ../mapping/start_mapping.py; exec bash"

# 2. ROS BRIDGE
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; $WS_SOURCE; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash" C-m

# 3. CAMERA STREAM
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "if [ -f ../classification/tcp_rasp.py ]; then python3 ../classification/tcp_rasp.py; elif [ -f tcp_rasp.py ]; then python3 tcp_rasp.py; else echo 'Error: tcp_rasp.py not found!'; fi; exec bash" C-m

# 4. MAP SAVER UTILITY
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; $WS_SOURCE; echo 'To save map run: ros2 run nav2_map_server map_saver_cli -f my_map'; exec bash" C-m

tmux select-layout -t $SESSION tiled

# Auto-attach so you see the results immediately
tmux attach -t $SESSION
