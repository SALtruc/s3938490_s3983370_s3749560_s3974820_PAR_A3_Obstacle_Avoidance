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

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESTART_SNAPS="${PROJECT_C_RESTART_SNAPS:-true}"
CHECK_ATTEMPTS="${PROJECT_C_CHECK_ATTEMPTS:-15}"
CHECK_SLEEP_SEC="${PROJECT_C_CHECK_SLEEP_SEC:-5}"

cd "$ROOT"

# shellcheck source=tools/rosbot_husarion_guard.sh
source "${ROOT}/tools/rosbot_husarion_guard.sh"

# ── helpers ────────────────────────────────────────────────────────────────────

snap_installed() { snap list "$1" >/dev/null 2>&1; }

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
  restart_snap husarion-depthai

  echo "[wait] Waiting 6 s for sensor topics to appear..."
  sleep 6
}

# ── 2. wait for all topics ─────────────────────────────────────────────────────

wait_for_full_fusion() {
  local attempt check_log exit_code

  for attempt in $(seq 1 "$CHECK_ATTEMPTS"); do
    echo "[step] Sensor check ${attempt}/${CHECK_ATTEMPTS}..."
    check_log="$(mktemp)"
    exit_code=0
    PROJECT_C_REQUIRE_FULL_FUSION="${PROJECT_C_REQUIRE_FULL_FUSION:-true}" \
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

echo "[step] Building Project C..."
bash tools/build_project_c.sh

wait_for_full_fusion

echo
echo "[step] Launching Project C full-fusion safety run..."
exec bash tools/run_project_c_safety.sh "$@"
