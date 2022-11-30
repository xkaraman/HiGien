#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash"

source "$HOME/ros2_ws/install/local_setup.bash"

exec "$@"