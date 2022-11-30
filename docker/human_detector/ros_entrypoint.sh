#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash"

source "/docker_ws/install/local_setup.bash"

# Folder must be mounted as a volume
source "/ros2_workspaces/higien_ws/install/local_setup.bash"
exec "$@"