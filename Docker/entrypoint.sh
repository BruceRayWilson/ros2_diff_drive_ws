#!/bin/bash

# Source the ROS environment - this adds all ROS commands to the PATH
source /opt/ros/jazzy/setup.bash

# Check if the workspace has a setup file and source it if it exists
if [ -f "/ros2_diff_drive_ws/install/setup.bash" ]; then
    source /ros2_diff_drive_ws/install/setup.bash
fi

# Execute the provided command (passes through any arguments given to the container)
exec "$@"
