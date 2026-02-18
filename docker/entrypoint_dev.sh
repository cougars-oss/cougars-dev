#!/bin/bash
# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

# Match UID/GID to local user
target_uid=$(stat -c '%u' "${CONFIG_FOLDER}")
target_gid=$(stat -c '%g' "${CONFIG_FOLDER}")
if [ -n "${target_gid}" ]; then
    groupmod -o -g "${target_gid}" "${DOCKER_USER}"
fi

if [ -n "${target_uid}" ]; then
    usermod -o -u "${target_uid}" "${DOCKER_USER}"
fi

chown "${DOCKER_USER}:${DOCKER_USER}" "/home/${DOCKER_USER}"
find "/home/${DOCKER_USER}" -maxdepth 1 -not -user "${DOCKER_USER}" \
    -exec chown -R "${DOCKER_USER}:${DOCKER_USER}" {} + 2>/dev/null || true

INIT_COMMAND="source /opt/ros/humble/setup.bash; \
              [ -f \"${OVERLAY_WS}/install/setup.bash\" ] && source \"${OVERLAY_WS}/install/setup.bash\"; \
              exec \"\$@\""

exec gosu "${DOCKER_USER}" bash -c "${INIT_COMMAND}" -- "$@"
