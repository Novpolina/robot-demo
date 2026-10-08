#!/usr/bin/env bash
# Record only the project's Gazebo/RViz displays; no personal desktop content is captured.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
if [[ ! -r install/setup.bash ]]; then bash scripts/build.sh; fi
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=robot_demo_video
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_QPA_PLATFORM=xcb
video_dir="${1:-artifacts/video}"
mkdir -p "$video_dir"
video_dir="$(realpath "$video_dir")"
if [[ -f "$video_dir/demo.mp4" ]]; then echo 'Existing video preserved; choose a new directory.' >&2; exit 2; fi
python3 - "$video_dir/record.rviz" <<'PY'
import sys,yaml
c=yaml.safe_load(open('src/robot_demo/config/demo.rviz'))
c['Panels']=[]
c['Visualization Manager']['Displays']=[d for d in c['Visualization Manager']['Displays'] if d['Class']!='rviz_default_plugins/Image']
c['Window Geometry'].update(Width=800,Height=850,X=0,Y=0)
yaml.safe_dump(c,open(sys.argv[1],'w'))
PY
cleanup() {
  [[ -n ${launch_pid:-} ]] && kill -INT -- "-$launch_pid" 2>/dev/null || true
  [[ -n ${rviz_pid:-} ]] && kill -INT "$rviz_pid" 2>/dev/null || true
  [[ -n ${launch_pid:-} ]] && wait "$launch_pid" 2>/dev/null || true
  [[ -n ${rviz_pid:-} ]] && wait "$rviz_pid" 2>/dev/null || true
  [[ -n ${display1:-} ]] && kill "$display1" 2>/dev/null || true
  [[ -n ${display2:-} ]] && kill "$display2" 2>/dev/null || true
}
trap cleanup EXIT
command -v xdpyinfo >/dev/null || { echo 'Install video dependencies with install_deps.sh --with-video.' >&2; exit 1; }
# Explicit free numbers also work with WSLg's read-only /tmp/.X11-unix mount;
# Xvfb -displayfd cannot allocate a listener on that mount.
start_display() {
  local label="$1" number candidate_pid
  for ((number=90;number<130;number++)); do
    [[ -e /tmp/.X${number}-lock ]] && continue
    xdpyinfo -display ":$number" >/dev/null 2>&1 && continue
    Xvfb ":$number" -screen 0 800x850x24 -nolisten tcp -ac > "$video_dir/xvfb-$label.log" 2>&1 & candidate_pid=$!
    sleep 0.3
    if kill -0 "$candidate_pid" 2>/dev/null && xdpyinfo -display ":$number" >/dev/null 2>&1; then
      new_display=":$number"; new_display_pid="$candidate_pid"; return
    fi
    kill "$candidate_pid" 2>/dev/null || true
    wait "$candidate_pid" 2>/dev/null || true
  done
  echo 'Could not start a private Xvfb display.' >&2; return 1
}
start_display gazebo
gazebo_display="$new_display"; display1="$new_display_pid"
start_display rviz
rviz_display="$new_display"; display2="$new_display_pid"
DISPLAY="$gazebo_display" setsid bash scripts/run.sh gui:=true rviz:=false start_delay:=15.0 report_dir:="$video_dir" > "$video_dir/launch.log" 2>&1 & launch_pid=$!
DISPLAY="$rviz_display" rviz2 -d "$video_dir/record.rviz" --ros-args -p use_sim_time:=true > "$video_dir/rviz.log" 2>&1 & rviz_pid=$!
sleep 8
ffmpeg -y -loglevel warning -f x11grab -draw_mouse 0 -video_size 800x850 -framerate 10 -i "$gazebo_display.0" -frames:v 1 "$video_dir/gazebo-preview.png"
ffmpeg -y -loglevel warning -f x11grab -draw_mouse 0 -video_size 800x850 -framerate 10 -i "$rviz_display.0" -frames:v 1 "$video_dir/rviz-preview.png"
ffmpeg -y -loglevel warning -thread_queue_size 128 -f x11grab -draw_mouse 0 -video_size 800x850 -framerate 10 -i "$gazebo_display.0" -thread_queue_size 128 -f x11grab -draw_mouse 0 -video_size 800x850 -framerate 10 -i "$rviz_display.0" \
  -filter_complex '[0:v][1:v]hstack=inputs=2,pad=1600:900:0:50:color=0x101824,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text=Gazebo Fortress - autonomous GPS mission:x=20:y=14:fontsize=22:fontcolor=white,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text=RViz2 - lidar + depth + map + path:x=820:y=14:fontsize=22:fontcolor=white' \
  -t 120 -c:v libx264 -threads 4 -preset veryfast -crf 23 -pix_fmt yuv420p "$video_dir/demo.mp4" > "$video_dir/ffmpeg.log" 2>&1
ffprobe -v error -show_entries format=duration,size -of json "$video_dir/demo.mp4"
python3 - "$video_dir/acceptance.json" <<'PY'
import json,sys
report=json.load(open(sys.argv[1]))
assert report['passed'], report
print('Video run acceptance: PASS')
PY
