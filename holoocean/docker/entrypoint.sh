#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs holoocean-ct'

set -e

# Fix permission errors
USERNAME=ue4
target_uid=$(stat -c '%u' /home/$USERNAME/config)
target_gid=$(stat -c '%g' /home/$USERNAME/config)

if [ ! -z "$target_gid" ]; then
    if [ "$target_gid" != "$(id -g $USERNAME)" ]; then
        echo "Changing GID of $USERNAME to $target_gid..."
        groupmod -o -g "$target_gid" $USERNAME
    fi
fi
if [ ! -z "$target_uid" ]; then
    if [ "$target_uid" != "$(id -u $USERNAME)" ]; then
        echo "Changing UID of $USERNAME to $target_uid..."
        usermod -o -u "$target_uid" $USERNAME
    fi
fi

# Source ROS environment
source /opt/ros/humble/setup.bash
if [ -f "/home/$USERNAME/ros2_ws/install/setup.bash" ]; then
    source "/home/$USERNAME/ros2_ws/install/setup.bash"
fi

touch /tmp/ready
exec "$@"
