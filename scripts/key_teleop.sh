#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Drives the BlueROV2 in HoloOcean using the keyboard

source "$(dirname "${BASH_SOURCE[0]}")/utils/common.sh"
source ${COLCON_WS}/install/setup.bash

print_info "Starting BlueROV2 keyboard teleop..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/blue0sim/cmd_vel
