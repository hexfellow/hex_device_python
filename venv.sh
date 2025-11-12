#!/usr/bin/env bash
set -Eeuo pipefail
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

CUR_DIR="$(pwd)"
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "CUR_DIR: $CUR_DIR"
echo "SCRIPT_DIR: $SCRIPT_DIR"

cd $SCRIPT_DIR

if ! command -v uv >/dev/null 2>&1; then
  echo "Error: uv not found. Please install uv first." >&2
  exit 1
fi

if [ ! -d .venv ]; then
  uv venv --python 3.9
fi
source .venv/bin/activate

# Install hex_zmq_servers
rm -rf dist build *.egg-info
uv pip install -e .

# install requirements for advanced examples
uv pip install -r requirements.txt

cd $CUR_DIR