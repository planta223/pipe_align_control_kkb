#!/usr/bin/env bash
# Record synchronized camera/vision/SDK feedback; motion requires --run-align.
set -eo pipefail
tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
control_pc_dir=$(cd -- "$tool_dir/.." && pwd)
run_align=0
record_seconds=
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-align) run_align=1; shift ;;
    --duration)
      [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]] || { echo '--duration requires positive seconds' >&2; exit 2; }
      record_seconds=$2; shift 2 ;;
    --help)
      echo 'Usage: bash main/control_pc/tools/diagnose_vision_align.sh [--run-align] [--duration seconds]'
      echo 'Default: record only. --run-align also runs vision_align --relearn --auto-start.'
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$record_seconds" ]]; then
  if [[ $run_align -eq 1 ]]; then record_seconds=330; else record_seconds=60; fi
fi
[[ $record_seconds -ge 1 && $record_seconds -le 600 ]] || { echo 'duration must be 1..600 seconds' >&2; exit 2; }
if [[ $run_align -eq 1 && ! -t 0 ]]; then
  echo '--run-align requires an interactive terminal.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$HOME/project/xarm_sdk/install/setup.bash"
source "$control_pc_dir/_colcon/install/local_setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-8}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
stamp=$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)
mkdir -p "$control_pc_dir/_log/vision_diagnostics"
record_dir=$(mktemp -d "$control_pc_dir/_log/vision_diagnostics/${stamp}_XXXXXX")
echo "기록 위치: $record_dir"
# Require actual messages, not just discovered names, before any control starts.
if ! python3 - <<'PY' > "$record_dir/preflight.log" 2>&1
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
rclpy.init()
node = rclpy.create_node('vision_recording_preflight')
topics = {'/cam/color/compressed': CompressedImage,
          '/cam/color/camera_info': CameraInfo,
          '/vision/left_pipe/pose': PoseStamped,
          '/vision/right_pipe/pose': PoseStamped}
seen = set()
subscriptions = [node.create_subscription(kind, topic, lambda m, t=topic: seen.add(t),
                qos_profile_sensor_data) for topic, kind in topics.items()]
end = time.monotonic() + 8
while seen != set(topics) and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.1)
missing = sorted(set(topics)-seen)
print('received:', sorted(seen))
print('missing:', missing)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(bool(missing))
PY
then
  cat "$record_dir/preflight.log"
  echo '카메라·FoundationPose 발행과 ROS 통신을 먼저 확인하세요. 제어기는 시작하지 않았습니다.' >&2
  exit 1
fi
cp "$control_pc_dir/config/control.yaml" "$record_dir/control.yaml"
cp "$control_pc_dir/config/fixed_frames.yaml" "$record_dir/fixed_frames.yaml"
recorder_pid=
cleanup() {
  if [[ -n "$recorder_pid" ]]; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
  if [[ -f "$record_dir/control_start.marker" ]]; then
    python3 - "$control_pc_dir/_log/auto_align" "$record_dir" <<'PY' || true
from pathlib import Path
import shutil, sys
source, destination = map(Path, sys.argv[1:])
started = (destination/'control_start.marker').stat().st_mtime_ns
for path in source.glob('auto_align_*.log'):
    if path.stat().st_mtime_ns >= started and '[auto_align]:' in path.read_text(errors='replace'):
        shutil.copy2(path, destination/path.name)
PY
  fi
  echo "진단 기록: $record_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
timeout --signal=INT --kill-after=5s "${record_seconds}s" ros2 bag record --storage sqlite3 \
  --output "$record_dir/bag" \
  /cam/color/compressed /cam/color/camera_info \
  /vision/left_pipe/pose /vision/right_pipe/pose \
  /auto_align/joint_states /auto_align/left/base_flange_pose /auto_align/right/base_flange_pose \
  /auto_align/status /auto_align/vision_diagnostics \
  > "$record_dir/recorder.log" 2>&1 < /dev/null &
recorder_pid=$!
sleep 1
if ! kill -0 "$recorder_pid" 2>/dev/null; then
  cat "$record_dir/recorder.log"
  echo '기록기를 시작하지 못했습니다. 제어기는 시작하지 않았습니다.' >&2
  exit 1
fi
if [[ $run_align -eq 1 ]]; then
  echo '영상·SDK·비전 기록을 시작했습니다. vision_align 신규 학습을 실행합니다.'
  touch "$record_dir/control_start.marker"
  set +e
  ros2 run auto_align auto_align --step vision_align --relearn --auto-start
  result=$?
  printf '%s\n' "$result" > "$record_dir/control_exit_code.txt"
  set -e
  exit "$result"
else
  echo "측정 기록만 ${record_seconds}초 수행합니다. 로봇 제어기를 시작하지 않습니다."
  set +e
  wait "$recorder_pid"
  result=$?
  set -e
  recorder_pid=
  [[ $result -eq 0 || $result -eq 124 || $result -eq 130 ]] || exit "$result"
fi
