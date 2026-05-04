#!/bin/bash
#
# Stop all Robot 2 (Beta) services
#

echo "Stopping Robot 2 services..."

# Kill by PID files
for svc in motors rosbridge camera; do
    pidfile="/tmp/robot2_${svc}.pid"
    if [ -f "$pidfile" ]; then
        pid=$(cat "$pidfile")
        kill $pid 2>/dev/null && echo "  ■ Stopped ${svc} (PID $pid)"
        rm -f "$pidfile"
    fi
done

# Fallback: kill by name
pkill -f professional_motor_controller.py 2>/dev/null
pkill -f tcp_rasp.py 2>/dev/null
pkill -f rosbridge_websocket 2>/dev/null

echo "✅ All Robot 2 services stopped."
