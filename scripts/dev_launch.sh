#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches FGO localization for HoloOcean development
#
# Usage:
#   ./scripts/dev_launch.sh [-b] [-c] [-m] [-r <bag_name>]
#
# Arguments:
#   -b: Launch the BlueROV2 model (default: CougUV)
#   -c: Launch comparison localization nodes
#   -m: Launch multiple CougUV agents
#   -r <bag_name>: Record a rosbag to ~/bags/<bag_name>

function printInfo {
    echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

source ~/coug_ws/install/setup.bash

URDF="urdf/couguv_holoocean.urdf.xacro"
AGENTS=1
BAG_PATH=""
COMPARE="false"

while getopts ":bcmr:" opt; do
    case $opt in
        b)
            URDF="urdf/bluerov2_holoocean/bluerov2_holoocean.urdf.xacro"
            ;;
        c)
            COMPARE="true"
            ;;
        m)
            AGENTS=3
            ;;
        r)
            BAG_PATH="$HOME/bags/$OPTARG"
            ;;
        \?)
            printError "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            printError "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

if [ -n "$BAG_PATH" ] && [ -d "$BAG_PATH" ]; then
    printError "Bag directory already exists: $BAG_PATH"
    exit 1
fi

ARGS=("urdf_file:=$URDF" "num_agents:=$AGENTS" "compare:=$COMPARE")
if [ -n "$BAG_PATH" ]; then
    ARGS+=("bag_path:=$BAG_PATH")
fi

ros2 launch coug_bringup dev.launch.py "${ARGS[@]}"