#!/bin/bash
set -e

# Source ROS and catkin workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Reduce USB latency for Dynamixel communication if the device exists
if [ -d "/sys/bus/usb-serial/devices" ]; then
    for dev in /sys/bus/usb-serial/devices/*/latency_timer; do
        if [ -f "$dev" ]; then
            echo 1 > "$dev" 2>/dev/null || true
        fi
    done
fi

exec "$@"
