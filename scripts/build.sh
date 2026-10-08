#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
python3 scripts/generate_world.py
colcon build --base-paths src --symlink-install
PYTHONPATH=src/robot_demo python3 -m pytest -q src/robot_demo/test
