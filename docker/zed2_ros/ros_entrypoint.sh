#!/bin/bash
set -e

echo "Start my initialization script..."

## Original entrypoint
ros_env_setup="/opt/ros/$ROS_DISTRO/install/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

# setup ros environment
ros_overlay_setup="$ROS_OVERLAY/install/local_setup.bash"
echo "sourcing   $ros_overlay_setup"
source "$ros_overlay_setup"
echo "ROS_OVERLAY   $ROS_OVERLAY"

exec "$@"