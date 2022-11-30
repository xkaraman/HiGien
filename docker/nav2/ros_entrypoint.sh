#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/ros/ros2_workspaces/higien_ws/install/local_setup.bash"

exec "$@"