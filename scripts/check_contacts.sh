#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
# Camera and GPU lidar still need an X/OpenGL context without GUI windows.
if [[ -z ${DISPLAY:-} ]]; then
  command -v xvfb-run >/dev/null && command -v xauth >/dev/null || {
    echo 'Install xvfb and xauth, or run scripts/install_deps.sh.' >&2; exit 1;
  }
  exec xvfb-run -a -s '-screen 0 1280x720x24' bash scripts/check_contacts.sh "$@"
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=robot_demo_contact_test
mkdir -p artifacts
setsid bash scripts/run.sh gui:=false rviz:=false start_delay:=100.0 spawn_x:=3.0 report_dir:="$(pwd)/artifacts/contact-test" > artifacts/contact-test.log 2>&1 &
launch_pid=$!
cleanup() { kill -INT -- "-$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; }
trap cleanup EXIT
python3 scripts/contact_probe.py
