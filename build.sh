docker build -t ros2_diff_drive_jazzy \
    --build-arg USERNAME=$(whoami) \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) .