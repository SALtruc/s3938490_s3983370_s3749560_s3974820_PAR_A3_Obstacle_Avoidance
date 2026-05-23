#!/usr/bin/env bash
# One-command Project C full-fusion run.
#
# This script:
#   1. restarts robot sensor/firmware snaps (rosbot first for STM32 micro-ROS),
#   2. builds this repository's package,
#   3. waits until LIDAR + OAK + ToF + odom + IMU topics are visible,
#   4. launches the safety controller.
#
# Tunable env vars:
#   PROJECT_C_RESTART_SNAPS=false   skip snap restart (if already running)
#   PROJECT_C_AUTO_INSTALL_SNAPS=true install missing Husarion snaps
#   PROJECT_C_ALLOW_NATIVE_HUSARION=true use already-running native drivers
#   PROJECT_C_ALLOW_NON_ROSBOT3=true continue if rosbot model is not rosbot
#   PROJECT_C_CHECK_ATTEMPTS=15     how many times to retry topic check
#   PROJECT_C_CHECK_SLEEP_SEC=5     seconds between retries
#   USE_TOF=false                   disable ToF (fallback if /range/* never appear)
#   AUTO_START_OAK=auto             auto-start native OAK driver when depthai snap is
#                                   disabled (auto/true/false).  Set false to disable.
#   CAMERA_MODEL=OAK-D-PRO          camera model passed to start_oak_pointcloud.sh
#   DEPTHAI_PACKAGE=depthai_ros_driver_v3   depthai ROS package
#   DEPTHAI_LAUNCH=driver.launch.py launch file within that package
#   DEPTH_TOPIC=...                 depth image topic (defaults to the v3 driver topic)
#   POINTCLOUD_TOPIC=/oak/points    pointcloud topic expected by the node

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESTART_SNAPS="${PROJECT_C_RESTART_SNAPS:-true}"
CHECK_ATTEMPTS="${PROJECT_C_CHECK_ATTEMPTS:-15}"
CHECK_SLEEP_SEC="${PROJECT_C_CHECK_SLEEP_SEC:-5}"

# OAK driver auto-start (used when husarion-depthai snap is disabled)
AUTO_START_OAK="${AUTO_START_OAK:-auto}"
CAMERA_MODEL="${CAMERA_MODEL:-OAK-D-PRO}"
DEPTHAI_PACKAGE="${DEPTHAI_PACKAGE:-depthai_ros_driver_v3}"
DEPTHAI_LAUNCH="${DEPTHAI_LAUNCH:-driver.launch.py}"
# depthai_ros_driver_v3 publishes under /camera/camera/... not /camera/...
# The pointcloud lands at depth/color/points, not the legacy /oak/points name.
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/depth/image_rect_raw}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/camera/camera/depth/color/points}"
# Export so run_project_c_safety.sh inherits the correct topic names.
export DEPTH_TOPIC POINTCLOUD_TOPIC

OAK_DRIVER_PID=''

cd "$ROOT"

# shellcheck source=tools/rosbot_husarion_guard.sh
source "${ROOT}/tools/rosbot_husarion_guard.sh"

# ── helpers ────────────────────────────────────────────────────────────────────

snap_installed() { snap list "$1" >/dev/null 2>&1; }

# True when the husarion-depthai snap exists but is disabled (not started by snap).
oak_snap_disabled() {
  command -v snap >/dev/null 2>&1 || return 0   # no snap → treat as absent → use native
  snap_installed husarion-depthai || return 0    # snap absent → use native
  local st
  st="$(snap services husarion-depthai 2>/dev/null | awk 'NR>1{print $2}' || echo unknown)"
  [ "$st" = "disabled" ]
}

should_auto_start_oak() {
  case "${AUTO_START_OAK,,}" in
    1|true|yes|on)   return 0 ;;
    0|false|no|off)  return 1 ;;
    *)               oak_snap_disabled ;;
  esac
}

_cleanup_oak() {
  if [ -n "$OAK_DRIVER_PID" ] && kill -0 "$OAK_DRIVER_PID" 2>/dev/null; then
    echo "[oak] stopping background OAK driver (PID $OAK_DRIVER_PID)..."
    kill "$OAK_DRIVER_PID" 2>/dev/null || true
    wait "$OAK_DRIVER_PID" 2>/dev/null || true
  fi
}
trap _cleanup_oak EXIT INT TERM

restart_snap() {
  local name="$1"
  if snap_installed "$name"; then
    echo "[snap] restarting $name..."
    sudo snap restart "$name" && echo "[ok]   $name restarted" \
      || echo "[warn] $name restart failed; continuing"
  else
    echo "[skip] $name not installed"
  fi
}

# ── 1. restart snaps ───────────────────────────────────────────────────────────

restart_snaps() {
  case "${RESTART_SNAPS,,}" in
    1|true|yes|on) ;;
    *) echo "[step] Snap restart skipped (PROJECT_C_RESTART_SNAPS=$RESTART_SNAPS)"; return 0 ;;
  esac

  if [ "${PROJECT_C_USING_SNAPS:-false}" != "true" ]; then
    echo "[step] Snap restart skipped (using native Husarion fallback)"
    return 0
  fi

  echo "[step] Restarting ROSbot sensor/firmware snaps..."
  sudo -v   # cache sudo credentials once

  # rosbot first — it resets the STM32 over GPIO and starts micro-ROS agent.
  # Everything else depends on the firmware being alive.
  restart_snap rosbot

  echo "[wait] Giving STM32 firmware 10 s to boot and connect via micro-ROS..."
  sleep 10

  restart_snap husarion-rplidar

  # Skip restarting the depthai snap when AUTO_START_OAK will launch the native
  # ROS driver instead (snap is disabled, or AUTO_START_OAK=true).
  if should_auto_start_oak; then
    echo "[oak] husarion-depthai snap is disabled — native driver will be started automatically."
  else
    restart_snap husarion-depthai
  fi

  echo "[wait] Waiting 6 s for sensor topics to appear..."
  sleep 6
}

# ── 1b. start native OAK driver in background if needed ───────────────────────

start_oak_background() {
  should_auto_start_oak || return 0

  local oak_wait=$(( CHECK_ATTEMPTS * CHECK_SLEEP_SEC + 30 ))
  echo "[oak] Launching depthai ROS driver in background..."
  echo "[oak] Logs → /tmp/oak_driver_bg.log"

  CAMERA_MODEL="$CAMERA_MODEL" \
  DEPTHAI_PACKAGE="$DEPTHAI_PACKAGE" \
  DEPTHAI_LAUNCH="$DEPTHAI_LAUNCH" \
  DEPTH_TOPIC="$DEPTH_TOPIC" \
  POINTCLOUD_TOPIC="$POINTCLOUD_TOPIC" \
  PROJECT_C_STOP_DEPTHAI_SNAP=true \
  WAIT_SEC="$oak_wait" \
    bash "${ROOT}/tools/start_oak_pointcloud.sh" \
    >/tmp/oak_driver_bg.log 2>&1 &
  OAK_DRIVER_PID=$!

  echo "[oak] OAK driver PID=$OAK_DRIVER_PID — waiting 15 s for USB enumeration..."
  sleep 15
}

# ── 2. wait for all topics ─────────────────────────────────────────────────────

wait_for_full_fusion() {
  local attempt check_log exit_code

  for attempt in $(seq 1 "$CHECK_ATTEMPTS"); do
    echo "[step] Sensor check ${attempt}/${CHECK_ATTEMPTS}..."
    check_log="$(mktemp)"
    exit_code=0
    PROJECT_C_REQUIRE_FULL_FUSION="${PROJECT_C_REQUIRE_FULL_FUSION:-true}" \
    DEPTH_TOPIC="$DEPTH_TOPIC" \
    POINTCLOUD_TOPIC="$POINTCLOUD_TOPIC" \
      bash tools/check_project_c_full.sh \
      2>&1 | tee "$check_log" || exit_code=$?

    if [ "$exit_code" -eq 0 ]; then
      rm -f "$check_log"
      return 0
    fi

    if [ "$attempt" -lt "$CHECK_ATTEMPTS" ]; then
      echo "[wait] Retrying in ${CHECK_SLEEP_SEC}s..."
      sleep "$CHECK_SLEEP_SEC"
    fi
  done

  echo
  echo "[error] Full-fusion topics not ready after $((CHECK_ATTEMPTS * CHECK_SLEEP_SEC))s."
  echo
  if [ "${PROJECT_C_USING_SNAPS:-false}" = "true" ] && command -v snap >/dev/null 2>&1; then
    echo "[diag] Snap service status:"
    snap services 2>/dev/null | grep -E 'rosbot|rplidar|depthai' || true
  else
    echo "[diag] Snap service status skipped (native Husarion fallback)"
  fi
  echo
  echo "[diag] Current range/ToF topics:"
  ros2 topic list 2>/dev/null | sort | grep -E 'range|tof|vl53|distance' || echo "  (none)"
  echo
  if [ "${PROJECT_C_USING_SNAPS:-false}" = "true" ] && command -v snap >/dev/null 2>&1; then
    echo "[diag] Recent rosbot snap logs:"
    sudo snap logs rosbot -n 60 | tail -30 || true
    echo
    echo "[hint] If /range/* never appear: run 'sudo rosbot.flash' then retry."
  fi
  echo "[hint] To run without ToF:  USE_TOF=false PROJECT_C_REQUIRE_FULL_FUSION=false bash $0"
  return 1
}

# ── main ───────────────────────────────────────────────────────────────────────

echo "[step] Checking Husarion ROSbot runtime..."
project_c_rosbot_husarion_guard

restart_snaps
start_oak_background

echo "[step] Building Project C..."
bash tools/build_project_c.sh

wait_for_full_fusion

echo
echo "[step] Launching Project C full-fusion safety run..."
exec bash tools/run_project_c_safety.sh "$@"
