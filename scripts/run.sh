#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble not found. Install dependencies with scripts/install_deps.sh.' >&2; exit 1
fi
if [[ -z ${DISPLAY:-} ]]; then
  echo 'No X11 display. For automated testing use scripts/check_run.sh (starts Xvfb).' >&2; exit 1
fi
export LANG="${LANG:-en_US.UTF-8}"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-robot_demo_${ROS_DOMAIN_ID}}"
# Reject a second run in the same domain/partition instead of mixing clocks and robot commands.
exec 9>"${TMPDIR:-/tmp}/robot-demo-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Another robot_demo run is already active.' >&2; exit 2; }
export QT_X11_NO_MITSHM=1
# WSLg's D3D12 OpenGL 4.1 driver cannot perform an Ogre2 texture copy used by Fortress.
# Mesa llvmpipe supports this operation; keep it overrideable on machines with native GL.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
if [[ ! -f install/setup.bash ]]; then colcon build --base-paths src --symlink-install; fi
source install/setup.bash
exec ros2 launch robot_demo demo.launch.py "$@"
