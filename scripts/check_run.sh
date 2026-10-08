#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
# Camera and GPU lidar still need an X/OpenGL context without GUI windows.
if [[ -z ${DISPLAY:-} ]]; then
  command -v xvfb-run >/dev/null && command -v xauth >/dev/null || {
    echo 'Install xvfb and xauth, or run scripts/install_deps.sh.' >&2; exit 1;
  }
  exec xvfb-run -a -s '-screen 0 1280x720x24' bash scripts/check_run.sh "$@"
fi
run_dir="${1:-artifacts/check-$(date -u +%Y%m%dT%H%M%S)-$$}"
mkdir -p "$run_dir"
run_dir="$(realpath "$run_dir")"
if [[ -f "$run_dir/acceptance.json" ]]; then
  echo "Choose a new report directory; existing acceptance.json is preserved." >&2; exit 2
fi
setsid bash scripts/run.sh gui:=false rviz:=false report_dir:="$run_dir" > "$run_dir/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
for ((i=0;i<210;i++)); do
  if [[ -f "$run_dir/acceptance.json" ]]; then
    python3 - "$run_dir/acceptance.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
print(json.dumps(r,indent=2))
sys.exit(0 if r['passed'] else 1)
PY
    exit $?
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then tail -60 "$run_dir/launch.log"; exit 1; fi
  if grep -q 'process has died' "$run_dir/launch.log"; then tail -40 "$run_dir/launch.log"; exit 1; fi
  sleep 1
done
echo 'Acceptance report was not produced.' >&2
tail -60 "$run_dir/launch.log"
exit 1
