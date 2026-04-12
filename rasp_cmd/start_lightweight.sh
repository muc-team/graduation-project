#!/bin/bash
#
# ╔═══════════════════════════════════════════════════════════════╗
# ║         ROBOT STARTUP SCRIPT - FIXED VERSION                  ║
# ║         LiDAR on /dev/ttyUSB1, Arduino on /dev/ttyUSB0        ║
# ╚═══════════════════════════════════════════════════════════════╝
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

clear
echo -e "${CYAN}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║           RESCUE ROBOT - AUTONOMOUS SYSTEM                    ║"
echo "║                  Lightweight Mode                             ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# ============================================
# Pre-flight Checks
# ============================================
echo -e "${YELLOW}[PREFLIGHT] Checking connections...${NC}"

# Fix permissions
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null

# Check Arduino (USB0)
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✅ Arduino detected on /dev/ttyUSB0${NC}"
else
    echo -e "${RED}❌ Arduino NOT found on /dev/ttyUSB0!${NC}"
    read -p "Press Enter to continue or Ctrl+C to abort..."
fi

# Check LiDAR (USB1)
if [ -e /dev/ttyUSB1 ]; then
    echo -e "${GREEN}✅ LiDAR detected on /dev/ttyUSB1${NC}"
else
    echo -e "${RED}❌ LiDAR NOT found on /dev/ttyUSB1!${NC}"
    read -p "Press Enter to continue or Ctrl+C to abort..."
fi

echo ""
echo -e "${GREEN}Starting all systems...${NC}"
sleep 1

# ============================================
# Step 1: LiDAR (on /dev/ttyUSB1)
# ============================================
gnome-terminal --tab --title="📡 LIDAR" -- bash -c "
source /opt/ros/humble/setup.bash
echo '📡 Starting LiDAR on /dev/ttyUSB1...'
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
exec bash"

echo "⏳ Waiting for LiDAR to start..."
sleep 4

# ============================================
# Step 2: SLAM (using slam_only.py - no LiDAR restart)
# ============================================
gnome-terminal --tab --title="🗺️ SLAM" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🗺️ Starting SLAM Toolbox...'
cd $DIR/../mapping
ros2 launch slam_only.py
exec bash"

sleep 2

# ============================================
# Step 3: Motor Controller (on /dev/ttyUSB0)
# ============================================
gnome-terminal --tab --title="⚙️ MOTOR" -- bash -c "
source /opt/ros/humble/setup.bash
echo '⚙️ Starting Smart Motor Controller...'
cd $DIR/../navigation
python3 smart_motor_controller.py
exec bash"

sleep 1

# ============================================
# Step 4: ROS Bridge (for PC Dashboard)
# ============================================
gnome-terminal --tab --title="🌐 ROS BRIDGE" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🌐 Starting ROS Bridge on port 9090...'
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"

sleep 1

# ============================================
# Step 5: Camera Stream (optional)
# ============================================
gnome-terminal --tab --title="📷 CAMERA" -- bash -c "
echo '📷 Starting Camera Stream on port 5555...'
cd $DIR/../classification
if [ -f tcp_rasp.py ]; then
    python3 tcp_rasp.py
else
    echo 'Camera script not found - skipping'
fi
exec bash"

# ============================================
# Step 6: Simple Explorer (disabled by default)
# ============================================
gnome-terminal --tab --title="🤖 EXPLORER" -- bash -c "
source /opt/ros/humble/setup.bash
echo '======================================'
echo '  AUTONOMOUS EXPLORER (disabled)'
echo '======================================'
echo ''
echo 'To START: ros2 topic pub /explore_enable std_msgs/Bool \"data: true\" -1'
echo 'To STOP:  ros2 topic pub /explore_enable std_msgs/Bool \"data: false\" -1'
echo ''
cd $DIR/../navigation
python3 simple_explorer.py
exec bash"

# ============================================
# Summary
# ============================================
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║               ✅ ALL SYSTEMS STARTED!                         ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}On PC, run:${NC}"
echo "  cd dashboard"
echo "  python dash_control.py"
echo ""
echo -e "${CYAN}Then open:${NC} http://localhost:8080"
echo ""
echo -e "${YELLOW}Port Configuration:${NC}"
echo "  Arduino:  /dev/ttyUSB0"
echo "  LiDAR:    /dev/ttyUSB1"
echo "  ROS:      robot.local:9090"
echo "  Camera:   robot.local:5555"
echo ""
echo -e "${RED}EMERGENCY STOP:${NC}"
echo "  ros2 topic pub /emergency_stop std_msgs/Bool 'data: true' -1"
echo ""
