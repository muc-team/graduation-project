#!/usr/bin/env python3
"""
Map Debug Tool - Check if map data is being received
"""

import roslibpy
import time

ROBOT = 'robot.local'

print("=" * 50)
print("MAP DEBUG TOOL")
print("=" * 50)

ros = roslibpy.Ros(host=ROBOT, port=9090)
map_received = False
map_count = 0

def on_map(msg):
    global map_received, map_count
    map_received = True
    map_count += 1
    w = msg['info']['width']
    h = msg['info']['height']
    res = msg['info']['resolution']
    print(f"✅ MAP #{map_count}: {w}x{h} pixels, resolution={res}m/pixel")

def on_ready():
    print(f"✅ Connected to ROS at {ROBOT}:9090")
    
    # List all topics
    print("\n📋 Getting topic list...")
    
    # Subscribe to map
    print("\n🗺️ Subscribing to /map topic...")
    map_topic = roslibpy.Topic(ros, '/map', 'nav_msgs/OccupancyGrid')
    map_topic.subscribe(on_map)
    print("   Waiting for map data...")

ros.on_ready(on_ready)

try:
    print(f"\n🔌 Connecting to {ROBOT}:9090...")
    ros.run()
except KeyboardInterrupt:
    print("\n\nStopped by user")
except Exception as e:
    print(f"\n❌ Connection error: {e}")
finally:
    if not map_received:
        print("\n" + "=" * 50)
        print("❌ NO MAP DATA RECEIVED!")
        print("=" * 50)
        print("\nPossible causes:")
        print("1. SLAM is not running on RPi")
        print("2. LiDAR is not working")
        print("3. ROS Bridge is not running")
        print("\nTo fix, run on RPi:")
        print("  cd ~/Desktop/graduation-project/rasp_cmd")
        print("  ./start_minimal.sh")
        print("\nThen check if SLAM is running:")
        print("  ros2 topic list | grep map")
        print("  ros2 topic echo /map --once")
