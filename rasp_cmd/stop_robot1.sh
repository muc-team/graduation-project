#!/bin/bash
#
# Stop all Robot 1 (Alpha) services
#

echo "Stopping Robot 1 services..."

# Kill by PID files
for svc in lidar slam motors rosbridge explorer; do
    pidfile="/tmp/robot1_${svc}.pid"
    if [ -f "$pidfile" ]; then
        pid=$(cat "$pidfile")
        kill $pid 2>/dev/null && echo "  ■ Stopped ${svc} (PID $pid)"
        rm -f "$pidfile"
    fi
done

# Fallback: kill by name
pkill -f robot1_bridge.py 2>/dev/null
pkill -f simple_explorer.py 2>/dev/null
pkill -f rplidar 2>/dev/null
pkill -f slam 2>/dev/null
pkill -f rosbridge_websocket 2>/dev/null

echo "✅ All Robot 1 services stopped."
