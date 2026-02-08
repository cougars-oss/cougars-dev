#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the configured scenario in HoloOcean
#
# Usage:
#   ./holoocean/compose.sh [down|up|command] [-b] [-m]
#
# Arguments:
#   down: Stop the holoocean-ct container
#   up: Start the holoocean-ct container and launch the CougUV scenario (default)
#   -b: Launch the BlueROV2 scenario
#   -m: Launch the multi-CougUV scenario
#   command: Any other arguments will be passed to the container (e.g. bash)

set -e

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/../scripts/common.sh"

case $1 in
    "down")
        print_warning "Stopping the holoocean-ct container..."
        docker compose -f "$script_dir/docker/docker-compose.yaml" down
        ;;
    "up" | "" | -*)
        if [[ "$1" == "up" ]]; then
            shift
        fi

        # Allow container to forward graphical displays to host
        xhost +local:root

        print_info "Loading the holoocean-ct container..."
        docker compose -f "$script_dir/docker/docker-compose.yaml" up -d

        # Wait for './entrypoint.sh' to finish
        while [ "$(docker exec holoocean-ct test -f /tmp/ready \
            && echo 'yes' || echo 'no')" != "yes" ]; do sleep 1; done

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
        docker exec -it --user ue4 -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/ue4/config/fastdds.xml holoocean-ct /bin/bash -c \
            "source ~/ros2_ws/install/setup.bash && ros2 run holoocean_main holoocean_node --ros-args \
            --params-file $params_file"
        ;;
    *)
        # Pass the command to the container
        docker exec -it --user ue4 holoocean-ct "$@"
        ;;
esac
