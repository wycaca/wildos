#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE="${REPO_ROOT}/.env.orin.wildos-cameras"
ENV_TEMPLATE="${REPO_ROOT}/.env.orin.wildos-cameras.example"
INVENTORY_FILE=""
ASSIGN_ROLE=""
REQUIRE_TRANSFORMS=false

usage() {
  cat <<'EOF'
Usage: deploy_orin_cameras.sh [options]

Options:
  --env-file PATH             Generated Compose environment file
  --inventory-file PATH       Read serial|model|usb_path records instead of librealsense
  --assign ROLE               Register one connected camera as front, left, or right
  --require-transforms        Reject missing calibrated transforms
  -h, --help                  Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env-file)
      ENV_FILE="${2:?Missing path for --env-file}"
      shift 2
      ;;
    --inventory-file)
      INVENTORY_FILE="${2:?Missing path for --inventory-file}"
      shift 2
      ;;
    --assign)
      ASSIGN_ROLE="${2:?Missing role for --assign}"
      ASSIGN_ROLE="${ASSIGN_ROLE,,}"
      shift 2
      ;;
    --require-transforms)
      REQUIRE_TRANSFORMS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${ENV_FILE}" ]]; then
  cp "${ENV_TEMPLATE}" "${ENV_FILE}"
  chmod 0600 "${ENV_FILE}"
  echo "Created Compose environment file: ${ENV_FILE}"
fi

set -a
source "${ENV_FILE}"
set +a

if [[ -n "${ASSIGN_ROLE}" && ! "${ASSIGN_ROLE}" =~ ^(front|left|right)$ ]]; then
  echo "Unsupported camera role: ${ASSIGN_ROLE}" >&2
  exit 1
fi

declare -A CAMERA_MODELS=()
declare -A CAMERA_PATHS=()
declare -A ASSIGNED_SERIALS=()

add_camera() {
  local serial="$1"
  local model="$2"
  local usb_path="$3"
  [[ -n "${serial}" ]] || return
  CAMERA_MODELS["${serial}"]="${model:-unknown}"
  CAMERA_PATHS["${serial}"]="${usb_path:-unknown}"
}

load_inventory_file() {
  local serial
  local model
  local usb_path
  while IFS='|' read -r serial model usb_path; do
    [[ -n "${serial}" ]] || continue
    [[ "${serial}" == \#* ]] && continue
    add_camera "${serial}" "${model}" "${usb_path}"
  done < "${INVENTORY_FILE}"
}

# Use the same librealsense serial number consumed by the ROS camera driver
discover_realsense_cameras() {
  local camera_image="wildos-cameras:orin-humble"
  local inventory
  local line
  local serial
  local model
  local usb_path

  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    inventory="$(rs-enumerate-devices)"
  elif docker image inspect "${camera_image}" >/dev/null 2>&1; then
    inventory="$(docker run --rm \
      --cap-drop=ALL \
      --device-cgroup-rule='c 189:* rmw' \
      --security-opt=no-new-privileges \
      -v /dev/bus/usb:/dev/bus/usb:rw \
      "${camera_image}" \
      bash -lc 'source /opt/ros/humble/setup.bash; rs-enumerate-devices')"
  else
    echo "Missing ${camera_image}, run scripts/wildos_docker.sh orin build cameras first" >&2
    exit 1
  fi

  while IFS= read -r line; do
    case "${line}" in
      *"Name"*":"*) model="${line#*:}" ;;
      *"Serial Number"*":"*) serial="${line#*:}" ;;
      *"Physical Port"*":"*)
        usb_path="${line#*:}"
        model="$(xargs <<<"${model:-}")"
        serial="$(xargs <<<"${serial:-}")"
        usb_path="$(xargs <<<"${usb_path}")"
        add_camera "${serial}" "${model}" "${usb_path}"
        serial=""
        model=""
        ;;
    esac
  done <<<"${inventory}"
}

if [[ -n "${INVENTORY_FILE}" ]]; then
  [[ -f "${INVENTORY_FILE}" ]] || {
    echo "Missing inventory file: ${INVENTORY_FILE}" >&2
    exit 1
  }
  load_inventory_file
else
  discover_realsense_cameras
fi

if [[ -n "${ASSIGN_ROLE}" && ${#CAMERA_MODELS[@]} -lt 1 ]]; then
  echo "Expected at least 1 Intel RealSense device, found 0" >&2
  exit 1
elif [[ -z "${ASSIGN_ROLE}" && ${#CAMERA_MODELS[@]} -ne 3 ]]; then
  echo "Expected exactly 3 Intel RealSense devices, found ${#CAMERA_MODELS[@]}" >&2
  exit 1
fi

mapfile -t CAMERA_SERIALS < <(printf '%s\n' "${!CAMERA_MODELS[@]}" | sort)

print_inventory() {
  local index
  local serial
  echo "Detected RealSense devices:"
  for index in "${!CAMERA_SERIALS[@]}"; do
    serial="${CAMERA_SERIALS[${index}]}"
    printf '  %d) serial=%s model=%s usb_path=%s\n' \
      "$((index + 1))" \
      "${serial}" \
      "${CAMERA_MODELS[${serial}]}" \
      "${CAMERA_PATHS[${serial}]} assigned=${ASSIGNED_SERIALS[${serial}]:-no}"
  done
}

find_serial_by_usb_path() {
  local path_fragment="$1"
  local serial
  local match=""
  [[ -n "${path_fragment}" ]] || return 1
  for serial in "${CAMERA_SERIALS[@]}"; do
    if [[ "${CAMERA_PATHS[${serial}]}" == *"${path_fragment}"* ]]; then
      if [[ -n "${match}" ]]; then
        echo "USB path matches multiple cameras: ${path_fragment}" >&2
        return 1
      fi
      match="${serial}"
    fi
  done
  [[ -n "${match}" ]] || return 1
  printf '%s\n' "${match}"
}

find_only_unassigned_camera() {
  local serial
  local match=""
  for serial in "${CAMERA_SERIALS[@]}"; do
    [[ -z "${ASSIGNED_SERIALS[${serial}]:-}" ]] || continue
    [[ -z "${match}" ]] || return 1
    match="${serial}"
  done
  [[ -n "${match}" ]] || return 1
  printf '%s\n' "${match}"
}

# Resolve roles from existing serials, stable USB paths, model hints, or operator input
select_camera() {
  local role="$1"
  local configured_serial="$2"
  local usb_path="$3"
  local serial=""
  local selection

  if [[ -n "${configured_serial}" && -n "${CAMERA_MODELS[${configured_serial}]:-}" ]]; then
    serial="${configured_serial}"
  elif serial="$(find_serial_by_usb_path "${usb_path}" 2>/dev/null)"; then
    :
  elif serial="$(find_only_unassigned_camera 2>/dev/null)"; then
    :
  elif [[ -t 0 ]]; then
    print_inventory >&2
    while true; do
      read -r -p "Select ${role} camera [1-${#CAMERA_SERIALS[@]}]: " selection
      if [[ "${selection}" =~ ^[0-9]+$ ]] \
        && ((selection >= 1 && selection <= ${#CAMERA_SERIALS[@]})); then
        serial="${CAMERA_SERIALS[$((selection - 1))]}"
        if [[ -n "${ASSIGNED_SERIALS[${serial}]:-}" ]]; then
          echo "Camera ${serial} is already assigned to ${ASSIGNED_SERIALS[${serial}]}" >&2
          continue
        fi
        break
      fi
    done
  else
    echo "Cannot resolve ${role} camera, configure ${role}_CAMERA_USB_PATH" >&2
    exit 1
  fi

  if [[ -n "${ASSIGNED_SERIALS[${serial}]:-}" ]]; then
    echo "Camera ${serial} is already assigned to ${ASSIGNED_SERIALS[${serial}]}" >&2
    exit 1
  fi
  ASSIGNED_SERIALS["${serial}"]="${role}"
  SELECTED_SERIAL="${serial}"
}

if [[ -n "${ASSIGN_ROLE}" ]]; then
  roles=("${ASSIGN_ROLE^^}")
  for role in FRONT LEFT RIGHT; do
    [[ "${role}" == "${roles[0]}" ]] && continue
    serial_variable="${role}_CAMERA_SERIAL"
    serial="${!serial_variable:-}"
    if [[ -n "${serial}" && -n "${CAMERA_MODELS[${serial}]:-}" ]]; then
      ASSIGNED_SERIALS["${serial}"]="${role}"
    fi
  done
else
  roles=(FRONT LEFT RIGHT)
fi

for role in "${roles[@]}"; do
  serial_variable="${role}_CAMERA_SERIAL"
  usb_path_variable="${role}_CAMERA_USB_PATH"
  select_camera "${role}" "${!serial_variable:-}" "${!usb_path_variable:-}"
  printf -v "${serial_variable}" '%s' "${SELECTED_SERIAL}"
done

# Update one dotenv key while preserving unrelated deployment settings
update_config_value() {
  local target_file="$1"
  local key="$2"
  local value="$3"
  local escaped_value
  local temporary_file
  escaped_value="${value//\\/\\\\}"
  escaped_value="${escaped_value//\"/\\\"}"
  temporary_file="$(mktemp)"
  awk -v key="${key}" -v value="${escaped_value}" '
    BEGIN { updated = 0 }
    $0 ~ "^" key "=" {
      print key "=\"" value "\""
      updated = 1
      next
    }
    { print }
    END {
      if (!updated) {
        print key "=\"" value "\""
      }
    }
  ' "${target_file}" > "${temporary_file}"
  mv "${temporary_file}" "${target_file}"
}

for role in "${roles[@]}"; do
  serial_variable="${role}_CAMERA_SERIAL"
  transform_variable="${role}_CAMERA_TRANSFORM"
  usb_path_variable="${role}_CAMERA_USB_PATH"
  serial="${!serial_variable}"
  update_config_value \
    "${ENV_FILE}" \
    "${usb_path_variable}" \
    "${CAMERA_PATHS[${serial}]}"
  update_config_value "${ENV_FILE}" "${serial_variable}" "${serial}"
done
chmod 0600 "${ENV_FILE}"

for role in "${roles[@]}"; do
  transform_variable="${role}_CAMERA_TRANSFORM"
  if [[ -z "${!transform_variable:-}" ]]; then
    if [[ "${REQUIRE_TRANSFORMS}" == "true" ]]; then
      echo "Missing calibrated transform: ${transform_variable}" >&2
      exit 1
    fi
    echo "Warning: ${transform_variable} is empty, image testing only" >&2
  fi
done

print_inventory
echo "Camera assignments:"
for role in "${roles[@]}"; do
  serial_variable="${role}_CAMERA_SERIAL"
  echo "  ${role,,}=${!serial_variable}"
done

if [[ -n "${ASSIGN_ROLE}" ]]; then
  echo "Registered ${ASSIGN_ROLE} camera in ${ENV_FILE}"
  exit 0
fi

echo "Camera deployment configuration is ready: ${ENV_FILE}"
echo "Run scripts/wildos_docker.sh orin update cameras to rebuild and restart cameras"
