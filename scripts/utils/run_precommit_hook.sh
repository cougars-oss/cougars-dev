#!/bin/bash
# Created by Nelson Durrant, Feb 2026

hook_name="$1"
shift
files=()

for f in "$@"; do
    files+=("src/${f#packages/}")
done

docker exec cougars-ct /bin/bash -c \
    "source /opt/ros/humble/setup.bash \
    && cd coug_ws \
    && export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
    && $hook_name \"\$@\"" -- "${files[@]}"
