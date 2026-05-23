#!/usr/bin/env bash
# Launch Project C full-fusion safety mode from this repository's install space.
#
# Control path:
#   - S2 LIDAR: /scan_filtered
#   - VL53L0X ToF: /range/fl,/range/fr,/range/rl,/range/rr
#   - OAK depth image and PointCloud2
# The Project C trial logger is launched automatically and writes CSV records
# under LOG_DIR/project_c_trial_<timestamp>/.
#
# Optional upload:
#   UPLOAD_RECORD_ON_EXIT=true      force-add, commit, and push this run's record
#   RECORD_UPLOAD_REMOTE=origin     git remote used for upload
#   RECORD_UPLOAD_BRANCH=<branch>   defaults to the current branch
#
# Sensor logging:
#   SENSOR_LOG_PERIOD_SEC=0.2       write sensor_fallback.csv at this period

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
# FastRTPS segfaults on the lab ROSbot image. Force CycloneDDS by default so
# running this script does not depend on the user's current shell exports.
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Keep this off by default because ROSbot firmware/sensor participants may be
# exposed through robot-local network namespaces instead of localhost.
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"
LAUNCH_PID=""

truthy() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

latest_record_dir() {
  local dir="${1%/}"
  if [ ! -d "$dir" ]; then
    return 1
  fi

  find "$dir" -maxdepth 1 -type d -name 'project_c_trial_*' -printf '%T@ %p\n' \
    2>/dev/null | sort -nr | sed -n '1s/^[^ ]* //p'
}

upload_record_dir() {
  local record_dir="$1"
  local remote="${RECORD_UPLOAD_REMOTE:-origin}"
  local branch="${RECORD_UPLOAD_BRANCH:-}"
  local repo_root
  local record_abs
  local commit_msg

  if [ -z "$record_dir" ] || [ ! -d "$record_dir" ]; then
    echo "[warn] No trial record directory found to upload."
    return 0
  fi

  if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[warn] Not inside a git worktree; skipping record upload."
    return 0
  fi

  repo_root="$(git rev-parse --show-toplevel)"
  record_abs="$(cd "$record_dir" && pwd -P)"
  case "$record_abs" in
    "$repo_root"/*) ;;
    *)
      echo "[warn] Record is outside the git repo: $record_dir"
      echo "[warn] Set LOG_DIR inside the repo if you want auto upload."
      return 0
      ;;
  esac

  if ! git diff --cached --quiet; then
    echo "[warn] Git already has staged changes; skipping auto record commit."
    echo "[warn] Commit/stash staged work first, then rerun upload manually."
    return 0
  fi

  echo "[step] Uploading trial record: $record_dir"
  if ! git add -f -- "$record_dir"; then
    echo "[warn] Failed to stage trial record."
    git reset -q -- "$record_dir" 2>/dev/null || true
    return 0
  fi

  if git diff --cached --quiet -- "$record_dir"; then
    echo "[warn] Trial record has no staged changes; skipping commit."
    return 0
  fi

  commit_msg="${RECORD_UPLOAD_COMMIT_MESSAGE:-upload_record_this_run: $(basename "$record_dir")}"
  if ! git commit -m "$commit_msg"; then
    echo "[warn] Failed to commit trial record."
    git reset -q -- "$record_dir" 2>/dev/null || true
    return 0
  fi

  if [ -z "$branch" ]; then
    branch="$(git branch --show-current)"
  fi
  if [ -z "$branch" ]; then
    echo "[warn] Detached HEAD; committed record locally but skipped push."
    return 0
  fi

  if git push "$remote" "HEAD:$branch"; then
    echo "[ok] Uploaded trial record to $remote/$branch"
  else
    echo "[warn] Commit created locally, but git push failed."
    echo "[warn] Check GitHub auth/network, then run: git push $remote HEAD:$branch"
  fi
}

forward_launch_signal() {
  local signal="$1"
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "-$signal" "$LAUNCH_PID" 2>/dev/null || true
  fi
}

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  echo "        Set ROS_DISTRO first, for example: ROS_DISTRO=jazzy bash $0"
  exit 1
fi

if [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[error] ${ROOT}/install/setup.bash not found."
  echo "        Build first: bash tools/build_project_c.sh"
  exit 2
fi

cd "$ROOT"

# Reset overlay variables so ~/ros2_ws cannot shadow this repo.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH

# Drop stale ROS library/python paths from previously sourced workspaces. The
# ROS setup files below will repopulate them in the right order.
unset LD_LIBRARY_PATH PYTHONPATH

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION

case "${PROJECT_C_LOCAL_ONLY,,}" in
  1|true|yes|on)
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
    ;;
  *)
    unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    ;;
esac

actual_prefix="$(ros2 pkg prefix rosbot_obstacle_avoidance 2>/dev/null || true)"
if [ "$actual_prefix" != "$EXPECTED_PREFIX" ]; then
  echo "[error] rosbot_obstacle_avoidance resolved to the wrong workspace:"
  echo "        actual  : ${actual_prefix:-<not found>}"
  echo "        expected: ${EXPECTED_PREFIX}"
  echo
  echo "        Rebuild first: bash tools/build_project_c.sh"
  exit 3
fi

echo "[ok] Using package: $actual_prefix"
echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[ok] PROJECT_C_LOCAL_ONLY=$PROJECT_C_LOCAL_ONLY"
CONTROL_USE_DEPTH="${CONTROL_USE_DEPTH:-${USE_DEPTH:-true}}"
CONTROL_USE_POINTCLOUD="${CONTROL_USE_POINTCLOUD:-${USE_POINTCLOUD:-true}}"
LOG_DEPTH="${LOG_DEPTH:-$CONTROL_USE_DEPTH}"
LOG_POINTCLOUD="${LOG_POINTCLOUD:-$CONTROL_USE_POINTCLOUD}"
LOG_DIR="${LOG_DIR:-record}"
TRIAL_ID="${TRIAL_ID:-project_c_trial_$(date +%Y%m%d_%H%M%S_%3N)}"
TRIAL_LABEL="${TRIAL_LABEL:-}"
SENSOR_CSV_ENABLED="${SENSOR_CSV_ENABLED:-true}"
SENSOR_LOG_PERIOD_SEC="${SENSOR_LOG_PERIOD_SEC:-0.2}"
SUMMARY_PERIOD_SEC="${SUMMARY_PERIOD_SEC:-5.0}"
ODOM_LOG_PERIOD_SEC="${ODOM_LOG_PERIOD_SEC:-0.2}"
LOG_OBSTACLE_SAMPLES="${LOG_OBSTACLE_SAMPLES:-false}"
OBSTACLE_SAMPLE_PERIOD_SEC="${OBSTACLE_SAMPLE_PERIOD_SEC:-0.2}"
UPLOAD_RECORD_ON_EXIT="${UPLOAD_RECORD_ON_EXIT:-false}"
if [ -z "$TRIAL_LABEL" ]; then
  if truthy "$CONTROL_USE_DEPTH" || truthy "$CONTROL_USE_POINTCLOUD"; then
    TRIAL_LABEL="full_fusion"
  elif truthy "${USE_TOF:-true}"; then
    TRIAL_LABEL="lidar_tof"
  else
    TRIAL_LABEL="lidar_only"
  fi
fi
if truthy "$CONTROL_USE_DEPTH" || truthy "$CONTROL_USE_POINTCLOUD"; then
  echo "[ok] oak_control=YES (OAK depth/pointcloud feeds obstacle perception)"
else
  echo "[ok] oak_control=NO (control uses LIDAR/ToF only)"
fi
if truthy "$LOG_DEPTH" || truthy "$LOG_POINTCLOUD"; then
  echo "[ok] oak_logging=YES (OAK depth/pointcloud included in sensor log)"
else
  echo "[ok] oak_logging=NO"
fi
if [ "${PROJECT_C_LOCAL_ONLY,,}" = "true" ] || [ "${PROJECT_C_LOCAL_ONLY}" = "1" ]; then
  echo "[ok] ROS discovery is restricted to localhost"
fi
echo "[ok] trial_logger=YES log_dir=$LOG_DIR trial_id=$TRIAL_ID trial_label=$TRIAL_LABEL"
echo "[ok] sensor_csv=$SENSOR_CSV_ENABLED sensor_log_period=${SENSOR_LOG_PERIOD_SEC}s"
echo "[ok] trial_logger_odom_period=${ODOM_LOG_PERIOD_SEC}s obstacle_samples=$LOG_OBSTACLE_SAMPLES"
echo "[ok] upload_record_on_exit=$UPLOAD_RECORD_ON_EXIT"

PRE_RUN_RECORD_DIR="$(latest_record_dir "$LOG_DIR" || true)"

set +e
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:="${SCAN_TOPIC:-/scan_filtered}" \
  depth_topic:="${DEPTH_TOPIC:-/camera/depth/image_rect_raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/oak/points}" \
  depth_qos:="${DEPTH_QOS:-auto}" \
  pointcloud_qos:="${POINTCLOUD_QOS:-auto}" \
  tof_topics:="${TOF_TOPICS:-/range/fl,/range/fr,/range/rl,/range/rr}" \
  tof_msg_type:="${TOF_MSG_TYPE:-laser_scan}" \
  front_tof_topics:="${FRONT_TOF_TOPICS:-/range/fl,/range/fr}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
  cmd_vel_stamped:="${CMD_VEL_STAMPED:-true}" \
  odom_topic:="${ODOM_TOPIC:-/rosbot_base_controller/odom}" \
  imu_topic:="${IMU_TOPIC:-/imu_broadcaster/imu}" \
  use_depth:="${CONTROL_USE_DEPTH}" \
  use_pointcloud:="${CONTROL_USE_POINTCLOUD}" \
  log_depth:="${LOG_DEPTH}" \
  log_pointcloud:="${LOG_POINTCLOUD}" \
  use_tof:="${USE_TOF:-true}" \
  use_nav2_collision_monitor:="${USE_NAV2_COLLISION_MONITOR:-false}" \
  local_only:="${PROJECT_C_LOCAL_ONLY}" \
  log_dir:="${LOG_DIR}" \
  trial_id:="${TRIAL_ID}" \
  trial_label:="${TRIAL_LABEL}" \
  sensor_csv_enabled:="${SENSOR_CSV_ENABLED}" \
  sensor_log_period_sec:="${SENSOR_LOG_PERIOD_SEC}" \
  summary_period_sec:="${SUMMARY_PERIOD_SEC}" \
  odom_log_period_sec:="${ODOM_LOG_PERIOD_SEC}" \
  log_obstacle_samples:="${LOG_OBSTACLE_SAMPLES}" \
  obstacle_sample_period_sec:="${OBSTACLE_SAMPLE_PERIOD_SEC}" \
  max_speed:="${MAX_SPEED:-0.22}" \
  observe_speed:="${OBSERVE_SPEED:-0.0}" \
  creep_speed:="${CREEP_SPEED:-0.14}" \
  clear_distance:="${CLEAR_DISTANCE:-0.38}" \
  front_release_distance:="${FRONT_RELEASE_DISTANCE:-0.40}" \
  rear_hard_stop_distance:="${REAR_HARD_STOP_DISTANCE:-0.05}" \
  rear_escape_side_clearance:="${REAR_ESCAPE_SIDE_CLEARANCE:-0.10}" \
  rear_escape_turn_sec:="${REAR_ESCAPE_TURN_SEC:-0.45}" \
  rear_escape_max_attempts:="${REAR_ESCAPE_MAX_ATTEMPTS:-4}" \
  side_guard_distance:="${SIDE_GUARD_DISTANCE:-0.045}" \
  side_escape_distance:="${SIDE_ESCAPE_DISTANCE:-0.07}" \
  corner_backup_side_distance:="${CORNER_BACKUP_SIDE_DISTANCE:-0.035}" \
  corner_backup_both_sides_distance:="${CORNER_BACKUP_BOTH_SIDES_DISTANCE:-0.07}" \
  backup_speed:="${BACKUP_SPEED:-0.06}" \
  backup_sec:="${BACKUP_SEC:-1.60}" \
  require_battery_ok:="${REQUIRE_BATTERY_OK:-false}" \
  min_battery_voltage:="${MIN_BATTERY_VOLTAGE:-5.0}" \
  warn_battery_voltage:="${WARN_BATTERY_VOLTAGE:-5.5}" \
  "$@" &
LAUNCH_PID=$!
trap 'forward_launch_signal INT' INT
trap 'forward_launch_signal TERM' TERM
while true; do
  wait "$LAUNCH_PID"
  launch_status=$?
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    continue
  fi
  break
done
trap - INT TERM
set -e

if truthy "$UPLOAD_RECORD_ON_EXIT"; then
  POST_RUN_RECORD_DIR="$(latest_record_dir "$LOG_DIR" || true)"
  if [ -n "$POST_RUN_RECORD_DIR" ] && [ "$POST_RUN_RECORD_DIR" != "$PRE_RUN_RECORD_DIR" ]; then
    upload_record_dir "$POST_RUN_RECORD_DIR"
  else
    echo "[warn] No new trial record directory detected; skipping upload."
  fi
fi

exit "$launch_status"
