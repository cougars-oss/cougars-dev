#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Sets up a quick CoUGARs development environment
#
# Usage:
#   ./compose.sh [down|up|command]
#
# Arguments:
#   down: Stop the cougars-ct container
#   up: Start the cougars-ct container (default)
#   command: Any other arguments will be passed to the container (e.g. bash)

set -e

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/scripts/common.sh"

profiles=""
arch=$(uname -m)
if [[ "$arch" == "x86_64" ]]; then
    profiles="--profile mapproxy"
fi

case $1 in
    "down")
        if [[ "$arch" == "x86_64" ]]; then
            print_warning "Stopping the mapproxy-ct container..."
        fi
        print_warning "Stopping the cougars-ct container..."
        docker compose -f "$script_dir/docker/docker-compose.yaml" $profiles down
        ;;
    "up" | "")
        # Allow container to forward graphical displays to host
        xhost +local:root

        if [[ "$arch" == "x86_64" ]]; then
            print_info "Loading the mapproxy-ct container..."
        fi
        print_info "Loading the cougars-ct container..."
        docker compose -f "$script_dir/docker/docker-compose.yaml" $profiles up -d

        # Wait for './entrypoint.sh' to finish
        while [ "$(docker exec cougars-ct test -f /tmp/ready \
            && echo 'yes' || echo 'no')" != "yes" ]; do sleep 1; done

        # Check if a 'coug_dev' tmux session already exists
        if ! docker exec --user frostlab-docker cougars-ct \
            tmux has-session -t coug_dev 2>/dev/null; then

            # If not, create a new 'coug_dev' tmux session
            print_warning "Creating a new 'coug_dev' tmux session..."
            docker exec -it --user frostlab-docker cougars-ct \
                tmuxp load -d /home/frostlab-docker/.tmuxp/coug_dev.yaml
        fi

        # Attach to the 'coug_dev' tmux session
        print_info "Attaching to the 'coug_dev' tmux session..."
        docker exec -it --user frostlab-docker cougars-ct \
            tmux attach -t coug_dev
        ;;
    *)
        # Pass the command to the container
        docker exec -it --user frostlab-docker cougars-ct "$@"
        ;;
esac
