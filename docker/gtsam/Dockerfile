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

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt update && apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libboost-dev \
    libtbb-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-date-time-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-regex-dev \
    ccache

RUN --mount=type=cache,target=/root/.ccache \
    ccache -z \
    && git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git \
    && cmake -B gtsam/build -S gtsam \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_WITH_TBB=ON \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && make -C gtsam/build -j$(nproc) \
    && sudo make -C gtsam/build install \
    && sudo ldconfig \
    && rm -rf gtsam \
    && ccache -s
