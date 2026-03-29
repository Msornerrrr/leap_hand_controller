#!/bin/bash
set -e

# Source catkin workspace (which also sources ROS Noetic)
source /catkin_ws/devel/setup.bash

# Set USB latency to 1ms for all USB-serial devices (critical for Dynamixel)
if [ -d "/sys/bus/usb-serial/devices" ]; then
    for dev in /sys/bus/usb-serial/devices/*/latency_timer; do
        [ -f "$dev" ] && echo 1 > "$dev" 2>/dev/null || true
    done
fi

exec "$@"
