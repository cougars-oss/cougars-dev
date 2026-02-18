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

source "$(dirname "$0")/utils/common.sh"

# --- Selection ---
scenario=""
if [ -z "$scenario" ]; then
    scenario=$(gum choose --header "Choose a HoloOcean scenario:" "CougUV" "BlueROV2" "Multi-Agent")
fi

case ${scenario} in
    "CougUV") scenario="couguv";;
    "BlueROV2") scenario="bluerov2";;
    "Multi-Agent") scenario="multiagent";;
esac

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Launch comparison methods" || true)

compare="false"
record_bag_path=""

if [[ "$options" == *"Launch comparison methods"* ]]; then
    compare="true"
fi

if [[ "$options" == *"Record rosbag"* ]]; then
    suffix=$(gum input --placeholder "Set bag suffix..." || echo "")
    if [ -n "$suffix" ]; then
        record_bag_path="${BAG_DIR}/${suffix}$(date +'_%Y-%m-%d-%H-%M-%S')"
    else
        record_bag_path="${BAG_DIR}/rosbag$(date +'_%Y-%m-%d-%H-%M-%S')"
    fi
fi

# --- Launch ---
args=("scenario:=$scenario" "compare:=$compare")
[ -n "$record_bag_path" ] && args+=("record_bag_path:=$record_bag_path")

echo "ros2 launch coug_bringup sim.launch.py ${args[*]}"
if [ -n "$record_bag_path" ]; then
    tmp=$(mktemp -d)
    ROS_LOG_DIR="${tmp}" ros2 launch coug_bringup sim.launch.py "${args[@]}"

    if [ -d "$record_bag_path" ]; then
        mv "${tmp}" "${record_bag_path}/log"

        mkdir -p "${record_bag_path}/config"
        src="${HOME}/config"
        if [ -d "${src}" ] && [ -n "$(ls -A "${src}")" ]; then
            cp -r "${src}/"* "${record_bag_path}/config/" 2>/dev/null || true
        fi
    else
        rm -rf "${tmp}"
    fi
else
    ros2 launch coug_bringup sim.launch.py "${args[@]}"
fi
