# Use ROS 2 Jazzy base image
FROM ros:jazzy

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-gazebo-* \
    && rm -rf /var/lib/apt/lists/*

# Create an entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Define the container mount point
WORKDIR /ros2_diff_drive_ws

# Use the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
