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

# Check for valid SSH keys
if ssh -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
    vcs import packages < cougars.repos
    vcs custom --git --args submodule update --init --recursive
else
    echo "WARNING: No valid SSH keys found for GitHub. Skipping VCS import."
    echo "To fix this, ensure your SSH agent is running and has your keys added."
fi

find . -maxdepth 4 -name '.pre-commit-config.yaml' -execdir pre-commit install \;
