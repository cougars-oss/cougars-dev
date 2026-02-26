#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash
source "$(dirname "$0")/utils/common.sh"

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" "CougUV" "BlueROV2" "Multi-Agent")

case ${scenario} in
    "CougUV") scenario="couguv";;
    "BlueROV2") scenario="bluerov2";;
    "Multi-Agent") scenario="multiagent";;
esac

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Launch comparison methods" "Disable sensor noise" || true)

compare="false"
record_bag_path=""
add_noise="true"

if [[ "$options" == *"Launch comparison methods"* ]]; then
    compare="true"
fi

if [[ "$options" == *"Disable sensor noise"* ]]; then
    add_noise="false"
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
args=("scenario:=$scenario" "compare:=$compare" "add_noise:=$add_noise")
[ -n "$record_bag_path" ] && args+=("record_bag_path:=$record_bag_path")

echo "ros2 launch coug_bringup sim.launch.py ${args[*]}"
if [ -n "$record_bag_path" ]; then
    tmp=$(mktemp -d)
    ROS_LOG_DIR="${tmp}" ros2 launch coug_bringup sim.launch.py "${args[@]}"

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
    ros2 launch coug_bringup sim.launch.py "${args[@]}"
fi
