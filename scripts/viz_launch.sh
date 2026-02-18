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
bag_name=$(cd "$BAG_DIR" && find . -maxdepth 3 -name "metadata.yaml" -exec dirname {} \; | sed 's|^\./||' | sort -r | gum filter --placeholder "Select a bag to play..." || exit 0)
[ -z "$bag_name" ] && exit 0
bag_path="$BAG_DIR/$bag_name"

agent_ns=$(printf "%s\n" "${!AGENTS[@]}" | sort | gum filter --placeholder "Select an agent to visualize..." || exit 0)
[ -z "$agent_ns" ] && exit 0

# --- Launch ---
args=("play_bag_path:=$bag_path" "auv_ns:=$agent_ns")
echo "ros2 launch coug_bringup viz.launch.py ${args[*]}"
ros2 launch coug_bringup viz.launch.py "${args[@]}"
