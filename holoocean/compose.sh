#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the configured scenario in HoloOcean
# Use the '-b' flag to launch the BlueROV2
# Use the '-m' flag to launch multiple CougUVs

set -e

function printInfo {
    # print blue
    echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
    # print yellow
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
    # print red
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

case $1 in
  	"down")
    	printWarning "Stopping the holoocean-ct container..."
    	docker compose -f $(dirname "$(readlink -f "$0")")/docker/docker-compose.yaml down
    	;;
  	*)
		# Allow container to forward graphical displays to host
		xhost +

		# Export host UID and GID for permission fixes
        export HOST_UID=$(id -u)
        export HOST_GID=$(id -g)

        printWarning "Starting the holoocean-ct container..."
        docker compose -f $(dirname "$(readlink -f "$0")")/docker/docker-compose.yaml up -d

        PARAMS_FILE="/home/ue4/config/coug_holoocean_params.yaml"
        while getopts ":bm" opt; do
            case $opt in
                b)
                    PARAMS_FILE="/home/ue4/config/bluerov2_holoocean_params.yaml"
                    ;;
                m)
                    PARAMS_FILE="/home/ue4/config/multi_coug_holoocean_params.yaml"
                    ;;
                \?)
                    echo "Invalid option: -$OPTARG" >&2
                    exit 1
                    ;;
            esac
        done

        docker exec -it --user ue4 -e HOME=/home/ue4 holoocean-ct /bin/bash -c "source ~/ros2_ws/install/setup.bash \
            && ros2 run holoocean_main holoocean_node --ros-args --params-file $PARAMS_FILE"
    ;;
esac
