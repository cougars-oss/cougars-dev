#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs cougars-ct'

set -e

# Fix permission errors
USERNAME=frostlab-docker
target_uid=$(stat -c '%u' /home/$USERNAME/coug_ws/src)
target_gid=$(stat -c '%g' /home/$USERNAME/coug_ws/src)

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

# Install vcs-defined external packages
current_dir=$(pwd)
cd /home/$USERNAME/coug_ws/src
if wget -q --spider http://github.com; then
    if [ -f "cougars.repos" ]; then
        echo "Network found. Updating vcs repositories..."
        gosu $USERNAME vcs import . < cougars.repos
        gosu $USERNAME vcs custom --git --args submodule update --init --recursive
    fi
else
    echo "No network connection. Skipping vcs repository updates."
fi
cd $current_dir

# Source ROS environment
source /opt/ros/humble/setup.bash
if [ -f "/home/$USERNAME/coug_ws/install/setup.bash" ]; then
    source "/home/$USERNAME/coug_ws/install/setup.bash"
fi

touch /tmp/ready
exec "$@"
