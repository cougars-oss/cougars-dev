#!/bin/bash
# Created by Nelson Durrant, Jan 2026

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HOOK="$1"; shift

for arg in "$@"; do
    if [[ "$arg" == -* ]]; then
        ARGS+=("$arg")
    else
        FILES+=("src/$(realpath --relative-to="$ROOT_DIR/packages" "$arg")")
    fi
done

docker exec cougars-ct /bin/bash -c \
    "source /opt/ros/humble/setup.bash && cd coug_ws \
    && \$0 ${ARGS[*]} \"\$@\"" "$HOOK" "${FILES[@]}"
