#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Pull the latest changes

docker pull snelsondurrant/cougars:latest
vcs pull $(dirname "$(readlink -f "$0")")/coug_ws/src
git pull
