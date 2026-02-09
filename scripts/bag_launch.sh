#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the CoUGARs development stack for a rosbag
#
# Usage:
#   ./scripts/bag_launch.sh <bag_name> [-c] [-d <seconds>] [-r <bag_name>]
#
# Arguments:
#   <bag_name>: Name of the rosbag to play (required)
#   -c: Launch alternative localization methods for comparison
#   -d <seconds>: Start offset in seconds
#   -r <bag_name>: Record a rosbag to ~/bags/<bag_name>

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/common.sh"
source "$script_dir/../coug_ws/install/setup.bash"

if [ -z "$1" ]; then
    print_error "Usage: $0 <bag_name> [-c] [-d <seconds>] [-r <bag_name>]"
    exit 1
fi

play_bag_path="$HOME/bags/$1"
shift

if [ ! -d "$play_bag_path" ]; then
    print_error "Bag not found: $play_bag_path"
    exit 1
fi

record_bag_path=""
compare="false"
delay="0.0"

while getopts ":cd:r:" opt; do
    case $opt in
        c)
            compare="true"
            ;;
        d)
            if [[ "$OPTARG" == -* ]]; then
                print_error "Option -d requires an argument." >&2
                exit 1
            fi
            delay="$OPTARG"
            ;;
        r)
            if [[ "$OPTARG" == -* ]]; then
                print_error "Option -r requires an argument." >&2
                exit 1
            fi
            timestamp=$(date +"_%Y-%m-%d-%H-%M-%S")
            record_bag_path="$HOME/bags/${OPTARG}${timestamp}"
            ;;
        \?)
            print_error "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            print_error "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

if [ -n "$record_bag_path" ] && [ -d "$record_bag_path" ]; then
    print_warning "Bag directory already exists: $record_bag_path"
    read -p "Do you want to overwrite it? (y/n) " -n 1 -r
    echo 
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$record_bag_path"
    else
        exit 1
    fi
fi

args=("compare:=$compare" "play_bag_path:=$play_bag_path" "start_delay:=$delay")
if [ -n "$record_bag_path" ]; then
    args+=("record_bag_path:=$record_bag_path")
fi

print_info "Launching development stack for bag: $play_bag_path..."
if [ -n "$record_bag_path" ]; then
    print_info "Recording to bag: $record_bag_path"

    temp_log_dir=$(mktemp -d)
    ROS_LOG_DIR="$temp_log_dir" ros2 launch coug_bringup bag.launch.py "${args[@]}"
    if [ -d "$record_bag_path" ] && [ -d "$temp_log_dir" ]; then
        mv "$temp_log_dir" "$record_bag_path/log"
    fi

    mkdir -p "$record_bag_path/config"
    find -L ~/coug_ws/install -type f \( -path "*/config/*" -o -path "*/rviz/*" -o -path "*/mapviz/*" -o -path "*/plotjuggler/*" \) -exec cp {} "$record_bag_path/config/" \;
else
    ros2 launch coug_bringup bag.launch.py "${args[@]}"
fi
