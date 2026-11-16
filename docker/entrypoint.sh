#!/bin/bash
set -e

# Fail-safe: alias python3\r to python3 in /usr/bin so Windows CRLF shebangs never crash ROS nodes
if [ ! -e "/usr/bin/python3"$'\r' ]; then
    ln -sf /usr/bin/python3 "/usr/bin/python3"$'\r' 2>/dev/null || true
fi

# Source official ROS Noetic environment
source "/opt/ros/noetic/setup.bash"

# Source custom Catkin workspace if compiled
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

exec "$@"
