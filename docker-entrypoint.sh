#!/usr/bin/env bash
# Entrypoint for the robotics-ws Docker image: sources ROS 2 and the built
# workspace overlays, then execs the requested command (default: bash).
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
exec "$@"
