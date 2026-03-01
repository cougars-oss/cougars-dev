#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
[ -f "${OVERLAY_WS}/install/setup.bash" ] && source "${OVERLAY_WS}/install/setup.bash"

exec "$@"
