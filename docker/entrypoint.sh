#!/bin/bash
set -e

# Source official ROS Noetic environment
source "/opt/ros/noetic/setup.bash"

# Source custom Catkin workspace if compiled
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

exec "$@"
