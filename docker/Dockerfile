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

# --- Cache Stage ---
FROM ubuntu:22.04 AS cacher

WORKDIR /tmp/packages
COPY packages .
RUN find . -type f \! \( -name "package.xml" -o -name "*.repos" \) -delete

# --- Main Stage ---
FROM snelsondurrant/cougars:gtsam

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG DOCKER_USER=frostlab-docker

ENV DOCKER_USER=$DOCKER_USER \
    ROS_BUILD_ARCH=$TARGETARCH \
    RCUTILS_COLORIZED_OUTPUT=1 \
    PATH="$PATH:/home/$DOCKER_USER/.local/bin"

# Install system dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt update && apt install -y --no-install-recommends \
    # Utilities
    gosu procps curl wget vim nano python3-pip python3-venv \
    openssh-server mosh tmux xclip \
    x11-apps x11-utils x11-xserver-utils xauth \
    # ROS Packages
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap

# Create user
RUN useradd -ms /bin/bash "$DOCKER_USER" \
    && usermod -aG sudo "$DOCKER_USER" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo "$DOCKER_USER:$DOCKER_USER" | chpasswd \
    && mkdir -p /home/$DOCKER_USER/ros2_ws/src \
    && chown -R $DOCKER_USER:$DOCKER_USER /home/$DOCKER_USER

# Set up SSH on port 2222
RUN mkdir -p /var/run/sshd \
    && ssh-keygen -A \
    && sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config \
    && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication no/' /etc/ssh/sshd_config \
    && sed -i 's/#PubkeyAuthentication yes/PubkeyAuthentication yes/' /etc/ssh/sshd_config \
    && echo "StrictModes no" >> /etc/ssh/sshd_config \
    && echo "Host *\n\tStrictHostKeyChecking no\n\tUserKnownHostsFile /dev/null" >> /etc/ssh/ssh_config

# Install Python dependencies
USER $DOCKER_USER
RUN --mount=type=cache,target=/home/$DOCKER_USER/.cache/pip,uid=1000,gid=1000 \
    pip3 install --upgrade vcstool tmuxp asciimatics Pillow

# Install tmux plugins
RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
COPY --chown=$DOCKER_USER:$DOCKER_USER docker/.tmux.conf /home/$DOCKER_USER/.tmux.conf
RUN ~/.tmux/plugins/tpm/bin/install_plugins || true

# Configure bash environment
RUN mkdir -p ~/.ssh && chmod 700 ~/.ssh \
    && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc \
    && echo '[ -f /completions.sh ] && source /completions.sh' >> ~/.bashrc \
    && touch ~/.hushlogin

# Install ROS dependencies (rosdep)
WORKDIR /home/$DOCKER_USER/ros2_ws/src
COPY --from=cacher --chown=$DOCKER_USER:$DOCKER_USER /tmp/packages .
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/$DOCKER_USER/.ros/rosdep,uid=1000,gid=1000 \
    sudo apt update \
    && rosdep update \
    && rosdep install --from-paths . --ignore-src -y \
    && rm -rf ./*

USER root
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/completions.sh /completions.sh
RUN chmod +x /entrypoint.sh /completions.sh

WORKDIR /home/$DOCKER_USER
ENTRYPOINT ["/entrypoint.sh"]
CMD gosu $DOCKER_USER /bin/bash
