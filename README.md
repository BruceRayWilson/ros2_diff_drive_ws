# Iron

## Dockerfile

The Docker file is no longer named 'Docker'.  It is now named 'Dockerfile'.

```docker
FROM ros:iron

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-iron-gazebo-* \
    && rm -rf /var/lib/apt/lists/*

# Add a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# Source ROS2 environment in bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Define the container mount point
VOLUME /ros2_diff_drive_ws
WORKDIR /ros2_diff_drive_ws

# Keep container running
CMD ["bash"]
```

## Docker Build

```bash
docker build -t ros2_diff_drive .
```
