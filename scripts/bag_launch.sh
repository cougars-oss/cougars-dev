#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash
source "$(dirname "$0")/utils/common.sh"

# --- Selection ---
bag_name=$(cd "$BAG_DIR" && find . -maxdepth 3 -name "metadata.yaml" -exec dirname {} \; | sed 's|^\./||' | sort -r | gum filter --placeholder "Select a bag to play..." || exit 0)
[ -z "$bag_name" ] && exit 0
bag_path="$BAG_DIR/$bag_name"

# agent_ns=$(printf "%s\n" "${!AGENTS[@]}" | sort | gum filter --placeholder "Select an agent to launch..." || exit 0)
# [ -z "$agent_ns" ] && exit 0
# auv_type="${AGENTS[$agent_ns]}"

agent_ns="bluerov2"
auv_type="bluerov2"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Set start delay" "Launch comparison methods" || true)

compare="false"
start_delay="0.0"
record_bag_path=""

if [[ "$options" == *"Launch comparison methods"* ]]; then
    compare="true"
fi

if [[ "$options" == *"Set start delay"* ]]; then
    start_delay=$(gum input --placeholder "Set start delay (s)..." || echo "0.0")
    if ! [[ "$start_delay" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        start_delay="0.0"
    fi
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
args=(
    "play_bag_path:=$bag_path"
    "auv_type:=$auv_type"
    "auv_ns:=$agent_ns"
    "compare:=$compare"
    "start_delay:=$start_delay"
)
[ -n "$record_bag_path" ] && args+=("record_bag_path:=$record_bag_path")

echo "ros2 launch coug_bringup bag.launch.py ${args[*]}"
if [ -n "$record_bag_path" ]; then
    tmp=$(mktemp -d)
    ROS_LOG_DIR="${tmp}" ros2 launch coug_bringup bag.launch.py "${args[@]}"

    if [ -d "$record_bag_path" ]; then
        mv "${tmp}" "${record_bag_path}/log"

        mkdir -p "${record_bag_path}/config"
        src="${HOME}/cougars-dev/config"
        if [ -d "${src}" ] && [ -n "$(ls -A "${src}")" ]; then
            cp -r "${src}/"* "${record_bag_path}/config/" 2>/dev/null || true
        fi
    else
        rm -rf "${tmp}"
    fi
else
    ros2 launch coug_bringup bag.launch.py "${args[@]}"
fi
