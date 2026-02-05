#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on devcontainer startup

git config --global --add safe.directory /workspace

cd /home/frostlab-docker/coug_ws/src
vcs import . < cougars.repos
vcs custom --git --args submodule update --init --recursive
