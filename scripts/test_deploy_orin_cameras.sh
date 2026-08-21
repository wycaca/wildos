#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEST_DIR="$(mktemp -d)"
ENV_FILE="${TEST_DIR}/cameras.env"
INVENTORY_FILE="${TEST_DIR}/inventory"

cleanup() {
  rm -rf "${TEST_DIR}"
}
trap cleanup EXIT

cp "${REPO_ROOT}/.env.orin.wildos-cameras" "${ENV_FILE}"
sed -i \
  -e 's/^FRONT_CAMERA_SERIAL=.*/FRONT_CAMERA_SERIAL=front/' \
  -e 's/^LEFT_CAMERA_SERIAL=.*/LEFT_CAMERA_SERIAL=left/' \
  -e 's/^RIGHT_CAMERA_SERIAL=.*/RIGHT_CAMERA_SERIAL=right/' \
  "${ENV_FILE}"

printf '%s\n' \
  'front|D435if|/sys/devices/usb2/2-3/2-3.1/2-3.1:1.0/video4linux/video2' \
  'left|D435i|/sys/devices/usb2/2-3/2-3.3/2-3.3:1.0/video4linux/video20' \
  'right|D435i|/sys/devices/usb2/2-3/2-3.2/2-3.2:1.0/video4linux/video8' \
  'head|D435i|/sys/devices/usb2/2-2/2-2:1.0/video4linux/video14' \
  > "${INVENTORY_FILE}"

bash "${SCRIPT_DIR}/deploy_orin_cameras.sh" \
  --env-file "${ENV_FILE}" \
  --inventory-file "${INVENTORY_FILE}" \
  > "${TEST_DIR}/output"

rg -q -Fx 'FRONT_CAMERA_USB_PATH="/sys/devices/usb2/2-3/2-3.1/2-3.1:1.0"' "${ENV_FILE}"
rg -q -Fx 'LEFT_CAMERA_USB_PATH="/sys/devices/usb2/2-3/2-3.3/2-3.3:1.0"' "${ENV_FILE}"
rg -q -Fx 'RIGHT_CAMERA_USB_PATH="/sys/devices/usb2/2-3/2-3.2/2-3.2:1.0"' "${ENV_FILE}"
rg -q -F 'serial=head' "${TEST_DIR}/output"
