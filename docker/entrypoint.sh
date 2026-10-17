#!/bin/bash
set -e

# Suppress ROS 1 EOL popup dialog
export DISABLE_ROS1_EOL_WARNINGS=1

# Source official ROS Noetic environment
source "/opt/ros/noetic/setup.bash"


# Source custom Catkin workspace if compiled
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

exec "$@"
