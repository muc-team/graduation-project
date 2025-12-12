#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

gnome-terminal --working-directory="$DIR" --title="SAVE MAP HERE" -- bash -c "echo 'To save map run: ros2 run nav2_map_server map_saver_cli -f my_map'; exec bash"

ros2 launch start_mapping.py
