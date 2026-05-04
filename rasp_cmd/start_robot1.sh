#!/bin/bash
#
# ╔═══════════════════════════════════════════════╗
# ║   ROBOT 1 (Alpha) - Clean Headless Startup   ║
# ║   LiDAR + SLAM + Motors + ROS Bridge          ║
# ║   No Camera (Robot 1 has no camera)           ║
# ╚═══════════════════════════════════════════════╝
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo "╔═══════════════════════════════════════════════╗"
echo "║       ROBOT 1 (Alpha) — Starting Up           ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║  Services:                                    ║"
echo "║    📡 LiDAR            (rplidar A1)           ║"
echo "║    🗺️  SLAM             (slam_toolbox)        ║"
echo "║    ⚙️  Motor Controller (Arduino serial)      ║"
echo "║    🌐 ROS Bridge       (WebSocket :9090)      ║"
echo "║    🤖 Explorer         (autonomous nav)       ║"
echo "╚═══════════════════════════════════════════════╝"
echo ""

# ── Kill any leftover processes from previous runs ──
echo "[CLEANUP] Stopping any previous services..."
pkill -f professional_motor_controller.py 2>/dev/null
pkill -f simple_explorer.py 2>/dev/null
pkill -f rplidar 2>/dev/null
pkill -f slam 2>/dev/null
pkill -f rosbridge_websocket 2>/dev/null
sleep 1

# ── Set serial port permissions ──
echo "[SETUP] Setting serial permissions..."
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null
sudo chmod 666 /dev/ttyACM0 2>/dev/null
sudo chmod 666 /dev/ttyACM1 2>/dev/null

# ── Helper: run command in background with logging ──
run_bg() {
    local name=$1
    local cmd=$2
    local logfile="/tmp/robot1_${name}.log"
    echo "  ▶ Starting ${name}... (log: ${logfile})"
    bash -c "source /opt/ros/humble/setup.bash; $cmd" > "${logfile}" 2>&1 &
    echo $! > "/tmp/robot1_${name}.pid"
}

# ═══════════════════════════════════════
# 1. LiDAR
# ═══════════════════════════════════════
run_bg "lidar" "ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1"
sleep 2

# ═══════════════════════════════════════
# 2. SLAM
# ═══════════════════════════════════════
run_bg "slam" "cd $DIR/../mapping && ros2 launch slam_only.py"
sleep 2

# ═══════════════════════════════════════
# 3. Motor Controller
# ═══════════════════════════════════════
run_bg "motors" "cd $DIR/../navigation && python3 professional_motor_controller.py"
sleep 1

# ═══════════════════════════════════════
# 4. ROS Bridge
# ═══════════════════════════════════════
run_bg "rosbridge" "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
sleep 1

# ═══════════════════════════════════════
# 5. Explorer (autonomous navigation)
# ═══════════════════════════════════════
run_bg "explorer" "cd $DIR/../navigation && python3 simple_explorer.py"

echo ""
echo "╔═══════════════════════════════════════════════╗"
echo "║         ✅ ROBOT 1 (Alpha) READY              ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║                                               ║"
echo "║  On your PC run:                              ║"
echo "║    python dashboard/dash_mapping.py           ║"
echo "║                                               ║"
echo "║  Then select 'Alpha' from the fleet panel     ║"
echo "║                                               ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║  View logs:                                   ║"
echo "║    tail -f /tmp/robot1_lidar.log              ║"
echo "║    tail -f /tmp/robot1_slam.log               ║"
echo "║    tail -f /tmp/robot1_motors.log             ║"
echo "║    tail -f /tmp/robot1_rosbridge.log          ║"
echo "║    tail -f /tmp/robot1_explorer.log           ║"
echo "║                                               ║"
echo "║  Stop everything:                             ║"
echo "║    bash rasp_cmd/stop_robot1.sh               ║"
echo "╚═══════════════════════════════════════════════╝"
echo ""
