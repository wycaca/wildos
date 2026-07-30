#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ $# -gt 0 ]]; then
  exec "$@"
fi

require_variable() {
  local variable_name="$1"
  if [[ -z "${!variable_name:-}" ]]; then
    echo "Missing camera variable: ${variable_name}" >&2
    exit 1
  fi
}

# Reject template values and duplicate physical device assignments
require_camera_serial() {
  local variable_name="$1"
  local serial
  require_variable "${variable_name}"
  serial="${!variable_name}"
  if [[ "${serial}" == replace_with_* ]]; then
    echo "Camera serial is still a template value: ${variable_name}" >&2
    exit 1
  fi
  if [[ ! "${serial}" =~ ^[0-9]+$ ]]; then
    echo "Camera serial must contain only digits: ${variable_name}" >&2
    exit 1
  fi
}

for variable_name in \
  FRONT_CAMERA_SERIAL \
  LEFT_CAMERA_SERIAL \
  RIGHT_CAMERA_SERIAL; do
  require_camera_serial "${variable_name}"
done

if [[ "${FRONT_CAMERA_SERIAL}" == "${LEFT_CAMERA_SERIAL}" ]] \
  || [[ "${FRONT_CAMERA_SERIAL}" == "${RIGHT_CAMERA_SERIAL}" ]] \
  || [[ "${LEFT_CAMERA_SERIAL}" == "${RIGHT_CAMERA_SERIAL}" ]]; then
  echo "Camera serial assignments must be unique" >&2
  exit 1
fi

camera_namespace="${CAMERA_NAMESPACE:-spot1/realsense}"
camera_width="${CAMERA_COLOR_WIDTH:-640}"
camera_height="${CAMERA_COLOR_HEIGHT:-480}"
camera_fps="${CAMERA_COLOR_FPS:-15}"
camera_pids=()

start_camera() {
  local name="$1"
  local serial="$2"
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:="${camera_namespace}" \
    camera_name:="${name}" \
    serial_no:="'${serial}'" \
    enable_color:=true \
    enable_depth:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_gyro:=false \
    enable_accel:=false \
    rgb_camera.color_profile:="${camera_width}x${camera_height}x${camera_fps}" \
    publish_tf:=true &
  camera_pids+=("$!")
}

# Publish calibrated base to camera link transforms when provided
start_static_tf() {
  local name="$1"
  local transform="$2"
  local transform_args=()
  if [[ -z "${transform}" ]]; then
    echo "Camera extrinsic is not configured: ${name}" >&2
    return
  fi
  read -r -a transform_args <<<"${transform}"
  if [[ ${#transform_args[@]} -ne 7 ]]; then
    echo "${name} camera transform must contain x y z qx qy qz qw" >&2
    exit 1
  fi
  ros2 run tf2_ros static_transform_publisher \
    --x "${transform_args[0]}" \
    --y "${transform_args[1]}" \
    --z "${transform_args[2]}" \
    --qx "${transform_args[3]}" \
    --qy "${transform_args[4]}" \
    --qz "${transform_args[5]}" \
    --qw "${transform_args[6]}" \
    --frame-id "${BASE_FRAME:-base_link}" \
    --child-frame-id "${name}_link" &
  camera_pids+=("$!")
}

stop_cameras() {
  if [[ ${#camera_pids[@]} -gt 0 ]]; then
    kill "${camera_pids[@]}" 2>/dev/null || true
    wait "${camera_pids[@]}" 2>/dev/null || true
  fi
}
trap stop_cameras EXIT INT TERM

start_camera front "${FRONT_CAMERA_SERIAL}"
start_camera left "${LEFT_CAMERA_SERIAL}"
start_camera right "${RIGHT_CAMERA_SERIAL}"

start_static_tf front "${FRONT_CAMERA_TRANSFORM:-}"
start_static_tf left "${LEFT_CAMERA_TRANSFORM:-}"
start_static_tf right "${RIGHT_CAMERA_TRANSFORM:-}"

wait -n "${camera_pids[@]}"
