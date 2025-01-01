# Use ROS 2 Jazzy base image
FROM ros:jazzy-perception

# Set environment variables for better maintainability
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Establish default arguments for username, UID, and GID in case they are not provided.
ARG USERNAME=rosuser
ARG USER_UID=$USER_UID
ARG USER_GID=$USER_GID

# Create user and group, install necessary packages
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Install sudo and add user to sudo group
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        python3-pip \
        python3-colcon-common-extensions \
        ros-jazzy-gazebo-* \
        nano \
    # Configure sudo access for user
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup apt cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Set up workspace directory
WORKDIR /ros2_diff_drive_ws

# Copy and set up entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && chown -R $USERNAME:$USERNAME /ros2_diff_drive_ws \
    && chown $USERNAME:$USERNAME /entrypoint.sh

# Switch to non-root user
USER $USERNAME

# Set up environment variables for the user
ENV HOME=/home/$USERNAME
ENV SHELL=/bin/bash

# Use the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]