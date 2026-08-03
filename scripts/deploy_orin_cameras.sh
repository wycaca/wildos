#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE="${REPO_ROOT}/.env.orin.wildos-cameras"
ENV_TEMPLATE="${REPO_ROOT}/.env.orin.wildos-cameras.example"
COMPOSE_FILE="${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
INVENTORY_FILE=""
PREPARE_ONLY=false
START_CAMERA=false
REQUIRE_TRANSFORMS=false

usage() {
  cat <<'EOF'
Usage: deploy_orin_cameras.sh [options]

Options:
  --env-file PATH             Generated Compose environment file
  --inventory-file PATH       Read serial|model|usb_path records instead of udev
  --prepare-only              Generate and validate configuration without building
  --up                        Start the cameras service after building
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
    --prepare-only)
      PREPARE_ONLY=true
      shift
      ;;
    --up)
      START_CAMERA=true
      shift
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

# Deduplicate the many video nodes exported by each physical RealSense device
discover_udev_cameras() {
  local device
  local properties
  local serial
  local model
  local usb_path
  local vendor_id

  if ! command -v udevadm >/dev/null 2>&1; then
    echo "Missing udevadm" >&2
    exit 1
  fi

  while IFS= read -r device; do
    properties="$(udevadm info --query=property --name="${device}" 2>/dev/null || true)"
    vendor_id="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"${properties}" | head -n 1)"
    [[ "${vendor_id,,}" == "8086" ]] || continue
    serial="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"${properties}" | head -n 1)"
    model="$(sed -n 's/^ID_MODEL=//p' <<<"${properties}" | head -n 1)"
    usb_path="$(sed -n 's/^ID_PATH=//p' <<<"${properties}" | head -n 1)"
    usb_path="${usb_path%-video-index*}"
    usb_path="${usb_path%:*}"
    add_camera "${serial}" "${model}" "${usb_path}"
  done < <(find /dev -maxdepth 1 -type c -name 'video*' -print | sort -V)
}

if [[ -n "${INVENTORY_FILE}" ]]; then
  [[ -f "${INVENTORY_FILE}" ]] || {
    echo "Missing inventory file: ${INVENTORY_FILE}" >&2
    exit 1
  }
  load_inventory_file
else
  discover_udev_cameras
fi

if [[ ${#CAMERA_MODELS[@]} -ne 3 ]]; then
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

find_unique_front_model() {
  local serial
  local model
  local match=""
  for serial in "${CAMERA_SERIALS[@]}"; do
    model="${CAMERA_MODELS[${serial}]^^}"
    [[ "${model}" == *"D435IF"* ]] || continue
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
  elif [[ "${role}" == "FRONT" ]] \
    && serial="$(find_unique_front_model 2>/dev/null)"; then
    :
  elif [[ -t 0 ]]; then
    print_inventory >&2
    while true; do
      read -r -p "Select ${role} camera [1-3]: " selection
      if [[ "${selection}" =~ ^[1-3]$ ]]; then
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

select_camera FRONT "${FRONT_CAMERA_SERIAL:-}" "${FRONT_CAMERA_USB_PATH:-}"
FRONT_CAMERA_SERIAL="${SELECTED_SERIAL}"
select_camera LEFT "${LEFT_CAMERA_SERIAL:-}" "${LEFT_CAMERA_USB_PATH:-}"
LEFT_CAMERA_SERIAL="${SELECTED_SERIAL}"
select_camera RIGHT "${RIGHT_CAMERA_SERIAL:-}" "${RIGHT_CAMERA_USB_PATH:-}"
RIGHT_CAMERA_SERIAL="${SELECTED_SERIAL}"

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

for role in FRONT LEFT RIGHT; do
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

for role in FRONT LEFT RIGHT; do
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
echo "  front=${FRONT_CAMERA_SERIAL}"
echo "  left=${LEFT_CAMERA_SERIAL}"
echo "  right=${RIGHT_CAMERA_SERIAL}"

compose=(docker compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}")
"${compose[@]}" config --quiet

if [[ "${PREPARE_ONLY}" == "true" ]]; then
  echo "Camera deployment configuration is ready: ${ENV_FILE}"
  exit 0
fi

"${compose[@]}" build cameras

if [[ "${START_CAMERA}" == "true" ]]; then
  "${compose[@]}" up -d cameras
fi

echo "Camera image build completed"
