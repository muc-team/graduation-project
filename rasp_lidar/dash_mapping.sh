#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

gnome-terminal --tab --title="MAPPING SYSTEM" -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch start_mapping.py;
exec bash"

gnome-terminal --tab --title="ROS BRIDGE (WIFI)" -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch rosbridge_server rosbridge_websocket_launch.xml;
exec bash"

gnome-terminal --tab --title="CAMERA STREAM" -- bash -c "
if [ -f classification/udp_rasp.py ]; then
    python3 classification/udp_rasp.py;
elif [ -f udp_rasp.py ]; then
    python3 udp_rasp.py;
else
    echo 'Error: udp_rasp.py not found!';
fi;
exec bash"

gnome-terminal --tab --title="SAVE MAP UTILITY" -- bash -c "
source /opt/ros/humble/setup.bash;
echo 'To save map run: ros2 run nav2_map_server map_saver_cli -f my_map';
exec bash"
