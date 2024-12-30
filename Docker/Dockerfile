# Use ROS 2 Jazzy base image
FROM ros:jazzy-perception

# Install necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-gazebo-* \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /ros2_diff_drive_ws

# Create an entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
