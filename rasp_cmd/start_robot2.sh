#!/bin/bash
#
# ╔═══════════════════════════════════════════════╗
# ║   ROBOT 2 (Beta) - Clean Headless Startup    ║
# ║   Camera + Motors + ROS Bridge only           ║
# ║   No LiDAR · No SLAM · No Explorer           ║
# ╚═══════════════════════════════════════════════╝
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo "╔═══════════════════════════════════════════════╗"
echo "║        ROBOT 2 (Beta) — Starting Up           ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║  Services:                                    ║"
echo "║    ⚙️  Motor Controller (Arduino serial)      ║"
echo "║    🌐 ROS Bridge      (WebSocket :9090)       ║"
echo "║    📷 Camera Stream   (ZMQ :5555)             ║"
echo "╚═══════════════════════════════════════════════╝"
echo ""

# ── Kill any leftover processes from previous runs ──
echo "[CLEANUP] Stopping any previous services..."
pkill -f robot2_bridge.py 2>/dev/null
pkill -f tcp_rasp.py 2>/dev/null
pkill -f rosbridge_websocket 2>/dev/null
sleep 1

# ── Set serial port permissions ──
echo "[SETUP] Setting serial permissions..."
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyACM0 2>/dev/null
sudo chmod 666 /dev/ttyACM1 2>/dev/null

# ── Helper: run command in background with logging ──
run_bg() {
    local name=$1
    local cmd=$2
    local logfile="/tmp/robot2_${name}.log"
    echo "  ▶ Starting ${name}... (log: ${logfile})"
    bash -c "source /opt/ros/humble/setup.bash; $cmd" > "${logfile}" 2>&1 &
    echo $! > "/tmp/robot2_${name}.pid"
}

# ═══════════════════════════════════════
# 1. Motor Controller (talks to Arduino)
# ═══════════════════════════════════════
run_bg "motors" "cd $DIR/../navigation && python3 robot2_bridge.py"
sleep 2

# ═══════════════════════════════════════
# 2. ROS Bridge (dashboard connects here)
# ═══════════════════════════════════════
run_bg "rosbridge" "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
sleep 2

# ═══════════════════════════════════════
# 3. Camera Stream (ZMQ video to dashboard)
# ═══════════════════════════════════════
run_bg "camera" "cd $DIR/../classification && python3 tcp_rasp.py"
sleep 1

echo ""
echo "╔═══════════════════════════════════════════════╗"
echo "║          ✅ ROBOT 2 (Beta) READY              ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║                                               ║"
echo "║  On your PC run:                              ║"
echo "║    python dashboard/dash_mapping.py           ║"
echo "║                                               ║"
echo "║  Then select 'Beta' from the fleet panel      ║"
echo "║                                               ║"
echo "╠═══════════════════════════════════════════════╣"
echo "║  View logs:                                   ║"
echo "║    tail -f /tmp/robot2_motors.log             ║"
echo "║    tail -f /tmp/robot2_rosbridge.log          ║"
echo "║    tail -f /tmp/robot2_camera.log             ║"
echo "║                                               ║"
echo "║  Stop everything:                             ║"
echo "║    bash rasp_cmd/stop_robot2.sh               ║"
echo "╚═══════════════════════════════════════════════╝"
echo ""
