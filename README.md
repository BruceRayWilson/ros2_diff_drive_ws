# Jazzy

## Dockerfile

The Docker file is no longer named 'Docker'.  It is now named 'Dockerfile'.

```docker
# This has been updated...
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
cd /home/wilsonb/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws/Docker/

docker build -t ros2_diff_drive_jazzy .
```

## Docker Run

```bash
docker run -it \
    --name ros2_diff_drive_jazzy \
    -v ~/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws:/ros2_diff_drive_ws \
    ros2_diff_drive_jazzy
```

## Docker Start

```bash
changed
docker start -i ros2_diff_drive
```

## Docker Remove Image

If necessary:

```bash
docker rmi -f ros2_diff_drive
```

## Upgrade Docker

```bash
sudo apt upgrade docker.io -y
```

## ROS2 Workspace

```bash
# Inside the container...
cd /ros2_diff_drive_ws/src

# Create a new package for the controller
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs nav_msgs tf2 tf2_ros control_msgs sensor_msgs test_msgs launch_py --description "Generic diff drive controller" --maintainer-email BruceRayWilson42@gmail.com --license MIT diff_drive_controller_generic


# Create a new package for the PWM control
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs sensor_msgs control_msgs launch_py --description "Generic PWM controller for robotics applications" --maintainer-email BruceRayWilson42@gmail.com --license MIT pwm_controller_generic

# Build the workspace
cd /ros2_diff_drive_ws
colcon build

```

### Updates

Update the description, maintainer, and license in the above nodes.

### Colcon Build Output

```text
Starting >>> diff_drive_controller_generic
Starting >>> pwm_controller_generic
Finished <<< pwm_controller_generic [0.62s]
Finished <<< diff_drive_controller_generic [0.63s]

Summary: 2 packages finished [0.71s]
```

## Commit Changes

1. Exit container
2. git add .
3. git commit -am "After colcon build."

## Create Differential Drive Controller Node

## Create PWM Node
