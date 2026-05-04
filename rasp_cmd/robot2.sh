#!/bin/bash
# ═══════════════════════════════════════════════
# ROBOT 2 (Beta) - Optimized Startup
# ═══════════════════════════════════════════════

SESSION="robot2"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_SOURCE="source /opt/ros/humble/setup.bash"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# 1. MOTOR BRIDGE (Talks to Arduino + IMU)
tmux new-session -d -s $SESSION -n "Motors" "$ROS_SOURCE; cd $DIR/../navigation && python3 robot2_bridge.py; exec bash"

# 2. ROS BRIDGE (For Dashboard connection)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash" C-m

# 3. CAMERA STREAM (ZMQ Video)
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $DIR/../classification && python3 tcp_rasp.py; exec bash" C-m

tmux select-layout -t $SESSION tiled

echo "✅ Robot 2 (Beta) Started in tmux session: $SESSION"
echo "To view logs: tmux attach -t $SESSION"
