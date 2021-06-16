#!/bin/bash
set -e

# setup ros environment
cd /
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"