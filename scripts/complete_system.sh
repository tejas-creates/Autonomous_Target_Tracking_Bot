#!/usr/bin/env bash

set -e

source /opt/ros/jazzy/setup.bash

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${PROJECT_DIR}/ros_ws/install/setup.bash"


ros2 launch "${PROJECT_DIR}/simulation/launch/world.launch.py" &
SIMULATION_PID=$!

sleep 2

ros2 launch "${PROJECT_DIR}/ros_ws/launch/simulation.launch.py" &
ROS_PID=$!

trap 'kill "$SIMULATION_PID" "$ROS_PID" 2>/dev/null' EXIT INT TERM

wait