#!/bin/bash
#
# ██████╗  ██████╗ ██████╗  ██████╗ ████████╗
# ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝
# ██████╔╝██║   ██║██████╔╝██║   ██║   ██║   
# ██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   
# ██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   
# ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝   
#
# Complete Autonomous Navigation System
# With Manual Override & Safety Features
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║          AUTONOMOUS ROBOT NAVIGATION SYSTEM                ║"
echo "║              with Manual Override                          ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Pre-flight checks
echo -e "${YELLOW}[PREFLIGHT] Checking system...${NC}"

# Check Arduino
if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}✅ Arduino detected${NC}"
else
    echo -e "${RED}❌ Arduino NOT detected!${NC}"
    echo "   Please connect Arduino via USB"
    read -p "Press Enter to continue anyway, or Ctrl+C to abort..."
fi

# Check LiDAR
if lsusb | grep -qi "CP210"; then
    echo -e "${GREEN}✅ LiDAR detected${NC}"
else
    echo -e "${YELLOW}⚠️  LiDAR not detected via USB (may be fine if using GPIO)${NC}"
fi

echo ""
echo -e "${GREEN}Starting all systems...${NC}"
echo ""

# =============================================
# Terminal 1: LiDAR
# =============================================
gnome-terminal --tab --title="📡 LIDAR" -- bash -c "
source /opt/ros/humble/setup.bash
echo '📡 Starting LiDAR...'
ros2 launch rplidar_ros rplidar_a1_launch.py
exec bash"
sleep 2

# =============================================
# Terminal 2: SLAM
# =============================================
gnome-terminal --tab --title="🗺️ SLAM" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🗺️ Starting SLAM Toolbox...'
cd $DIR/../mapping
ros2 launch start_mapping.py
exec bash"
sleep 2

# =============================================
# Terminal 3: Motor Controller
# =============================================
gnome-terminal --tab --title="⚙️ MOTOR CTRL" -- bash -c "
source /opt/ros/humble/setup.bash
echo '⚙️ Starting Smart Motor Controller...'
echo '   Manual override: /manual_cmd'
echo '   Emergency stop: /emergency_stop'
cd $DIR/../navigation
python3 smart_motor_controller.py
exec bash"
sleep 1

# =============================================
# Terminal 4: ROS Bridge (for external dashboard)
# =============================================
gnome-terminal --tab --title="🌐 ROS BRIDGE" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🌐 Starting ROS Bridge for WebSocket...'
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"
sleep 1

# =============================================
# Terminal 5: Camera Stream
# =============================================
gnome-terminal --tab --title="📷 CAMERA" -- bash -c "
echo '📷 Starting Camera Stream...'
cd $DIR/../classification
if [ -f tcp_rasp.py ]; then
    python3 tcp_rasp.py
else
    echo 'Camera script not found'
fi
exec bash"

# =============================================
# Terminal 6: Control Center Dashboard
# =============================================
gnome-terminal --tab --title="🎮 CONTROL CENTER" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🎮 Starting Control Center Dashboard...'
echo ''
echo '╔═══════════════════════════════════════════════╗'
echo '║  Open browser: http://localhost:8888          ║'
echo '║  Or from PC:   http://192.168.1.7:8888       ║'
echo '╚═══════════════════════════════════════════════╝'
echo ''
cd $DIR/../navigation
python3 control_center.py
exec bash"

# =============================================
# Terminal 7: Simple Explorer (optional)
# =============================================
gnome-terminal --tab --title="🤖 EXPLORER" -- bash -c "
source /opt/ros/humble/setup.bash
echo '🤖 Simple Explorer Ready'
echo ''
echo 'To START autonomous exploration:'
echo '  ros2 topic pub /explore_enable std_msgs/Bool \"data: true\" -1'
echo ''
echo 'To STOP autonomous exploration:'
echo '  ros2 topic pub /explore_enable std_msgs/Bool \"data: false\" -1'
echo ''
cd $DIR/../navigation
python3 simple_explorer.py
exec bash"

# =============================================
# Summary
# =============================================
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║               ALL SYSTEMS STARTED!                         ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Control Center:${NC} http://localhost:8888"
echo ""
echo -e "${YELLOW}Quick Commands:${NC}"
echo "  Start Exploration:  ros2 topic pub /explore_enable std_msgs/Bool 'data: true' -1"
echo "  Stop Exploration:   ros2 topic pub /explore_enable std_msgs/Bool 'data: false' -1"
echo "  Emergency Stop:     ros2 topic pub /emergency_stop std_msgs/Bool 'data: true' -1"
echo ""
echo -e "${RED}IN CASE OF EMERGENCY:${NC}"
echo "  1. Press Ctrl+C in this terminal"
echo "  2. Or unplug USB cable"
echo "  3. Or run: ros2 topic pub /emergency_stop std_msgs/Bool 'data: true' -1"
echo ""
