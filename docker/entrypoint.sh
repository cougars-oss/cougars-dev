#!/bin/bash
# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

if [ "$CI" == "true" ]; then
    echo "CI environment detected. Skipping..."
    exec "$@"
fi

# Fix permissions
DOCKER_USER=${DOCKER_USER}
target_uid=$(stat -c '%u' /home/$DOCKER_USER/ros2_ws/src)
target_gid=$(stat -c '%g' /home/$DOCKER_USER/ros2_ws/src)

if [ ! -z "$target_gid" ]; then
    if [ "$target_gid" != "$(id -g $DOCKER_USER)" ]; then
        echo "Changing GID of $DOCKER_USER to $target_gid..."
        groupmod -o -g "$target_gid" $DOCKER_USER
    fi
fi
if [ ! -z "$target_uid" ]; then
    if [ "$target_uid" != "$(id -u $DOCKER_USER)" ]; then
        echo "Changing UID of $DOCKER_USER to $target_uid..."
        usermod -o -u "$target_uid" $DOCKER_USER
    fi
fi

find "/home/$DOCKER_USER" \
    -maxdepth 1 \
    -not -user "$DOCKER_USER" \
    -exec chown -R $DOCKER_USER:$DOCKER_USER {} + 2>/dev/null || true

# Install external ROS packages (vcs)
git config --system --add safe.directory "*"
if curl -s --head --connect-timeout 5 https://github.com | grep "200" > /dev/null; then
    echo "Updating vcs repositories..."
    gosu $DOCKER_USER vcs import /home/$DOCKER_USER/ros2_ws/src < /home/$DOCKER_USER/ros2_ws/src/packages.repos
    gosu $DOCKER_USER vcs custom /home/$DOCKER_USER/ros2_ws/src --git --args submodule update --init --recursive
else
    echo "No network connection. Skipping..."
fi

# Set up SSH (host keys)
if [ -d "/home/$DOCKER_USER/.ssh" ]; then
    cat /home/$DOCKER_USER/.ssh/*.pub >> /home/$DOCKER_USER/.ssh_internal/authorized_keys 2>/dev/null || true
    sort -u /home/$DOCKER_USER/.ssh_internal/authorized_keys -o /home/$DOCKER_USER/.ssh_internal/authorized_keys
fi

# Start the SSH server
if [ -x /usr/sbin/sshd ]; then
    echo "Starting SSH server on port 2222..."
    /usr/sbin/sshd
fi

touch /tmp/ready
exec "$@"
