#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Syncs Jupyter notebooks using Jupytext

source "$(dirname "$0")/../scripts/common.sh"
source "$(dirname "$0")/../.venv/bin/activate"

jupytext --to notebook "$(dirname "$0")/plots/fgo_batch.py"
jupytext --to notebook "$(dirname "$0")/plots/fgo_analysis.py"
jupytext --to notebook "$(dirname "$0")/diagrams/fgo_diagrams.py"
