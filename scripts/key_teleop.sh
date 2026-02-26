#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash
source "$(dirname "$0")/utils/common.sh"

# agent_ns=$(printf "%s\n" "${!AGENTS[@]}" | sort | gum filter --placeholder "Select an agent to drive..." || exit 0)
# [ -z "$agent_ns" ] && exit 0

agent_ns="blue0sim"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:="/${agent_ns}/cmd_vel"
