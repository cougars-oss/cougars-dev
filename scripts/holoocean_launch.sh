#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the selected scenario in HoloOcean
#
# Usage:
#   ./holoocean/launch.sh [-b] [-m]
#
# Arguments:
#   -b: Launch the BlueROV2 scenario
#   -m: Launch the multi-CougUV scenario

set -e

source "$(dirname "${BASH_SOURCE[0]}")/utils/common.sh"

params_file="/home/ue4/config/coug_holoocean_params.yaml"
while getopts ":bm" opt; do
    case $opt in
        b)
            params_file="/home/ue4/config/bluerov2_holoocean_params.yaml"
            ;;
        m)
            params_file="/home/ue4/config/multi_coug_holoocean_params.yaml"
            ;;
        \?)
            print_error "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

print_info "Launching the configured scenario in HoloOcean..."
docker exec -it --user ue4 holoocean-ct /bin/bash -c \
    "source /opt/ros/humble/setup.bash && source /home/ue4/ros2_ws/install/setup.bash \
    && ros2 run holoocean_main holoocean_node --ros-args --params-file $params_file"
