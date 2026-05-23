#!/usr/bin/env bash
# Guard helpers for running Project C on a Husarion ROSbot stack.
#
# This file is safe to source from other scripts. When executed directly, it
# runs the guard once and exits with the guard status.

project_c_truthy() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

project_c_warn_native_fallback() {
  echo "[warn] Continuing with the machine's existing Husarion/ROS stack."
  echo "[warn] Project C can run this way only if the required topics already match."
  echo "[warn] The topic check will verify that before motion is launched."
}

project_c_snap_installed() {
  command -v snap >/dev/null 2>&1 && snap list "$1" >/dev/null 2>&1
}

project_c_snap_channel() {
  local snap_name="$1"
  local distro="${ROS_DISTRO:-jazzy}"
  local global_channel="${PROJECT_C_SNAP_CHANNEL:-}"

  if [ -n "$global_channel" ]; then
    printf '%s\n' "$global_channel"
    return 0
  fi

  case "$snap_name:$distro" in
    rosbot:jazzy) printf '%s\n' "${PROJECT_C_ROSBOT_SNAP_CHANNEL:-jazzy/edge}" ;;
    husarion-depthai:jazzy) printf '%s\n' "${PROJECT_C_DEPTHAI_SNAP_CHANNEL:-jazzy/edge}" ;;
    husarion-rplidar:jazzy) printf '%s\n' "${PROJECT_C_RPLIDAR_SNAP_CHANNEL:-jazzy/stable}" ;;
    rosbot:humble) printf '%s\n' "${PROJECT_C_ROSBOT_SNAP_CHANNEL:-humble/stable}" ;;
    husarion-depthai:humble) printf '%s\n' "${PROJECT_C_DEPTHAI_SNAP_CHANNEL:-humble/stable}" ;;
    husarion-rplidar:humble) printf '%s\n' "${PROJECT_C_RPLIDAR_SNAP_CHANNEL:-humble/stable}" ;;
    *) printf '%s\n' "" ;;
  esac
}

project_c_install_snap_if_missing() {
  local snap_name="$1"
  local channel

  if project_c_snap_installed "$snap_name"; then
    echo "[ok] snap installed: $snap_name"
    return 0
  fi

  if ! project_c_truthy "${PROJECT_C_AUTO_INSTALL_SNAPS:-true}"; then
    echo "[error] Required snap '$snap_name' is not installed."
    echo "        Install it or rerun with PROJECT_C_AUTO_INSTALL_SNAPS=true."
    return 1
  fi

  channel="$(project_c_snap_channel "$snap_name")"
  if [ -n "$channel" ]; then
    echo "[snap] sudo snap install $snap_name --channel=$channel"
    sudo snap install "$snap_name" --channel="$channel" || return 1
  else
    echo "[snap] sudo snap install $snap_name"
    sudo snap install "$snap_name" || return 1
  fi

  if [ "$snap_name" = "rosbot" ] && [ -x /var/snap/rosbot/common/post_install.sh ]; then
    echo "[snap] sudo /var/snap/rosbot/common/post_install.sh"
    sudo /var/snap/rosbot/common/post_install.sh || true
  fi

  return 0
}

project_c_configure_rosbot_snap() {
  local expected_model="${PROJECT_C_EXPECT_ROSBOT_MODEL:-rosbot}"
  local model

  model="$(snap get rosbot driver.robot-model 2>/dev/null | tr -d '"' | tail -n 1 || true)"
  model="${model##* }"

  if [ -z "$model" ] || [ "$model" = "-" ]; then
    echo "[snap] driver.robot-model is unset; setting it to '$expected_model'"
    sudo snap set rosbot "driver.robot-model=$expected_model"
    model="$expected_model"
  fi

  if [ "$model" != "$expected_model" ]; then
    echo "[warn] rosbot snap driver.robot-model=$model, expected $expected_model for ROSbot 3/3 PRO style runs."
    if project_c_truthy "${PROJECT_C_ALLOW_NON_ROSBOT3:-false}"; then
      echo "[warn] PROJECT_C_ALLOW_NON_ROSBOT3=true; continuing and relying on topic compatibility."
    else
      echo "[error] This run is configured for Husarion ROSbot 3/3 PRO style topics."
      echo "        If this robot still publishes the required topics, rerun with:"
      echo "        PROJECT_C_ALLOW_NON_ROSBOT3=true bash tools/run_project_c_full.sh"
      return 1
    fi
  else
    echo "[ok] rosbot snap model: $model"
  fi
}

project_c_sync_snap_ros_args() {
  if [ ! -f /var/snap/rosbot/common/ros_snap_args ]; then
    return 0
  fi

  case "${PROJECT_C_SYNC_SNAP_ROS:-true}" in
    1|true|yes|on) ;;
    *) return 0 ;;
  esac

  for snap_name in husarion-depthai husarion-rplidar; do
    if project_c_snap_installed "$snap_name"; then
      echo "[snap] syncing ROS args from rosbot to $snap_name"
      sudo snap set "$snap_name" $(xargs -a /var/snap/rosbot/common/ros_snap_args) || true
    fi
  done
}

project_c_configure_snap_transport() {
  local rmw="${PROJECT_C_RMW_IMPLEMENTATION:-${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}"
  local transport=""
  local snap_name

  case "$rmw" in
    rmw_cyclonedds_cpp) transport="rmw_cyclonedds_cpp" ;;
    rmw_fastrtps_cpp|rmw_fastdds_cpp) transport="rmw_fastrtps_cpp" ;;
    *) return 0 ;;
  esac

  case "${PROJECT_C_CONFIGURE_SNAP_RMW:-true}" in
    1|true|yes|on) ;;
    *) return 0 ;;
  esac

  for snap_name in rosbot husarion-depthai husarion-rplidar; do
    if project_c_snap_installed "$snap_name"; then
      echo "[snap] setting $snap_name ros.transport=$transport"
      sudo snap set "$snap_name" "ros.transport=$transport" || true
    fi
  done
}

project_c_source_rosbot_env_if_available() {
  local had_nounset=false

  if [ ! -f /var/snap/rosbot/common/ros.env ]; then
    return 0
  fi

  case "${PROJECT_C_SOURCE_ROSBOT_ENV:-true}" in
    1|true|yes|on)
      case "$-" in
        *u*) had_nounset=true ;;
      esac
      set +u
      # shellcheck source=/dev/null
      source /var/snap/rosbot/common/ros.env
      if [ "$had_nounset" = true ]; then
        set -u
      fi
      echo "[ok] Sourced Husarion snap ROS env: /var/snap/rosbot/common/ros.env"
      ;;
  esac
}

project_c_rosbot_husarion_guard() {
  local allow_native="${PROJECT_C_ALLOW_NATIVE_HUSARION:-false}"
  local required_snaps=(rosbot husarion-rplidar husarion-depthai)
  local snap_name

  export PROJECT_C_USING_SNAPS=false

  if ! command -v snap >/dev/null 2>&1; then
    echo "[warn] snap command not found."
    if project_c_truthy "$allow_native"; then
      project_c_warn_native_fallback
      return 0
    fi
    echo "[error] This script expects Husarion ROSbot snaps."
    echo "        To use already-running native Husarion drivers, set:"
    echo "        PROJECT_C_ALLOW_NATIVE_HUSARION=true"
    return 1
  fi

  sudo -v || return 1

  for snap_name in "${required_snaps[@]}"; do
    project_c_install_snap_if_missing "$snap_name" || return 1
  done

  project_c_configure_rosbot_snap || return 1
  project_c_configure_snap_transport
  project_c_sync_snap_ros_args
  project_c_source_rosbot_env_if_available

  export PROJECT_C_USING_SNAPS=true
  echo "[ok] Husarion ROSbot snap stack is available."
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  set -euo pipefail
  project_c_rosbot_husarion_guard
fi
