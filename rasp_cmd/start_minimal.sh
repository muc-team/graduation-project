#!/bin/bash
#
# ╔═══════════════════════════════════════════════════════════════╗
# ║              RASPBERRY PI - MINIMAL MODE                       ║
# ║         Only essential services, all processing on PC          ║
# ╚═══════════════════════════════════════════════════════════════╝
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# Colors
G='\033[0;32m'
Y='\033[1;33m'
C='\033[0;36m'
R='\033[0;31m'
N='\033[0m'

clear
echo -e "${C}"
cat << "EOF"
  ____  _____ ____   ____ _   _ _____   ____   ___  ____   ___ _____ 
 |  _ \| ____/ ___| / ___| | | | ____| |  _ \ / _ \| __ ) / _ \_   _|
 | |_) |  _| \___ \| |   | | | |  _|   | |_) | | | |  _ \| | | || |  
 |  _ <| |___ ___) | |___| |_| | |___  |  _ <| |_| | |_) | |_| || |  
 |_| \_\_____|____/ \____|\___/|_____| |_| \_\\___/|____/ \___/ |_|  
                                                                     
EOF
echo -e "${N}"
echo -e "${Y}Minimal Mode - All heavy processing on PC${N}"
echo ""

# ============================================
# Permissions
# ============================================
echo -e "${Y}[1/5] Setting permissions...${N}"
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null

# ============================================
# Check devices
# ============================================
echo -e "${Y}[2/5] Checking devices...${N}"

if [ -e /dev/ttyUSB0 ]; then
    echo -e "  ${G}✓${N} Arduino on /dev/ttyUSB0"
else
    echo -e "  ${R}✗${N} Arduino not found"
fi

if [ -e /dev/ttyUSB1 ]; then
    echo -e "  ${G}✓${N} LiDAR on /dev/ttyUSB1"
else
    echo -e "  ${R}✗${N} LiDAR not found"
fi

echo ""

# ============================================
# Start LiDAR
# ============================================
echo -e "${Y}[3/5] Starting LiDAR...${N}"
gnome-terminal --tab --title="📡 LiDAR" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
exec bash" &
sleep 3

# ============================================
# Start SLAM
# ============================================
echo -e "${Y}[4/5] Starting SLAM...${N}"
gnome-terminal --tab --title="🗺️ SLAM" -- bash -c "
source /opt/ros/humble/setup.bash
cd $DIR/../mapping
ros2 launch slam_only.py
exec bash" &
sleep 2

# ============================================
# Start Motor Controller
# ============================================
echo -e "${Y}[5/5] Starting Motor Controller...${N}"
gnome-terminal --tab --title="⚙️ Motor" -- bash -c "
source /opt/ros/humble/setup.bash
cd $DIR/../navigation
python3 professional_motor_controller.py
exec bash" &
sleep 1

# ============================================
# Start ROS Bridge
# ============================================
gnome-terminal --tab --title="🌐 Bridge" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash" &
sleep 1

# ============================================
# Start Camera
# ============================================
gnome-terminal --tab --title="📷 Camera" -- bash -c "
cd $DIR/../classification
python3 tcp_rasp.py
exec bash" &

# ============================================
# Done!
# ============================================
echo ""
echo -e "${G}╔═══════════════════════════════════════════════════════════════╗${N}"
echo -e "${G}║                    ✅ ROBOT READY!                            ║${N}"
echo -e "${G}╚═══════════════════════════════════════════════════════════════╝${N}"
echo ""
echo -e "${C}On your PC, run:${N}"
echo ""
echo "    cd dashboard"
echo "    python professional_dashboard.py"
echo ""
echo -e "${C}Then open:${N} http://localhost:8080"
echo ""
echo -e "${Y}Ports:${N}"
echo "    Arduino:  /dev/ttyUSB0"
echo "    LiDAR:    /dev/ttyUSB1"
echo "    ROS:      9090"
echo "    Camera:   5555"
echo ""
