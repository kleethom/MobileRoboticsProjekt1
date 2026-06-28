#!/bin/bash

echo "Starting turtlesimAutomata..."

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$PROJECT_DIR/ros2_ws"

if ! lsb_release -d | grep -q "Ubuntu 22.04"; then
    echo "Warning: This project was designed for Ubuntu 22.04.5 LTS."
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble was not found."
    echo "Please install ROS2 Humble before running this script."
    exit 1
fi

source /opt/ros/humble/setup.bash

cd "$WORKSPACE" || exit 1

echo "Installing missing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y

echo "Building complete workspace..."
colcon build --symlink-install --executor sequential

source install/setup.bash

echo "Launching turtlesimAutomata..."
ros2 launch turtlesimAutomata automata.launch.py