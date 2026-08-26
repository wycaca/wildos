#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEST_DIR="$(mktemp -d)"
CALLS_FILE="${TEST_DIR}/docker-calls"
ENV_FILE="${TEST_DIR}/x86.env"
ORIN_ENV_FILE="${TEST_DIR}/orin.env"

cleanup() {
  rm -rf "${TEST_DIR}"
}
trap cleanup EXIT

# Runtime services must not regain broad host privileges
! rg -q 'privileged:[[:space:]]*true' \
  "${REPO_ROOT}/compose.orin.wildos-cameras.yaml" \
  "${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q 'c 189:\* rmw' "${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
rg -q 'c 81:\* rmw' "${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
rg -q '/run/udev:/run/udev:ro' "${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
test "$(rg -c 'no-new-privileges:true' \
  "${REPO_ROOT}/compose.orin.wildos-cameras.yaml")" -eq 2
test "$(rg -c 'no-new-privileges:true' \
  "${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml")" -eq 2
rg -q '^  navigation:' "${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q 'wildos-navigation:x86_64' "${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q '/wildos_navigation/config/navigation.yaml:/config/navigation.yaml:ro' \
  "${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q 'export ROS_DOMAIN_ID=0' "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
rg -q 'NetworkInterface name="eno1"' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
rg -Fq 'systemctl disable "${SERVICE_NAME}"' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
! rg -Fq 'start|stop|restart|status)' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
rg -Fq 'systemctl show "${SERVICE_NAME}"' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
rg -Fq 'sudo -n /usr/bin/systemctl "$1" "${SERVICE_NAME}"' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
rg -Fq 'wildos-go2-motion-gateway.sudoers.in' \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"
test "$(rg -o '/usr/bin/systemctl (start|stop|restart) wildos-go2-motion-gateway.service' \
  "${REPO_ROOT}/systemd/wildos-go2-motion-gateway.sudoers.in" | wc -l)" -eq 3
! rg -q 'NOPASSWD:[[:space:]]*ALL' \
  "${REPO_ROOT}/systemd/wildos-go2-motion-gateway.sudoers.in"
bash -n \
  "${REPO_ROOT}/docker/entrypoint.navigation.x86_64.sh" \
  "${REPO_ROOT}/docker/healthcheck.navigation.x86_64.sh" \
  "${REPO_ROOT}/scripts/go2_motion_gateway.sh"

docker() {
  printf '%s\n' "$*" >> "${CALLS_FILE}"
}
export -f docker
export CALLS_FILE

cp "${REPO_ROOT}/.env.x86_64.lidar-dlio" "${ENV_FILE}"

bash "${SCRIPT_DIR}/wildos_docker.sh" x86 update --env-file "${ENV_FILE}" localization

compose_prefix="compose --env-file ${ENV_FILE} -f ${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q -Fx "${compose_prefix} config --quiet" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} build --pull localization" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} up --detach --force-recreate localization" "${CALLS_FILE}"

cp "${REPO_ROOT}/.env.orin.wildos-cameras" "${ORIN_ENV_FILE}"
: > "${CALLS_FILE}"

bash "${SCRIPT_DIR}/wildos_docker.sh" orin start --env-file "${ORIN_ENV_FILE}" cameras
bash "${SCRIPT_DIR}/wildos_docker.sh" orin stop --env-file "${ORIN_ENV_FILE}" cameras
bash "${SCRIPT_DIR}/wildos_docker.sh" orin restart --env-file "${ORIN_ENV_FILE}" cameras
bash "${SCRIPT_DIR}/wildos_docker.sh" orin logs --env-file "${ORIN_ENV_FILE}" -f cameras
bash "${SCRIPT_DIR}/wildos_docker.sh" orin down --env-file "${ORIN_ENV_FILE}"

compose_prefix="compose --env-file ${ORIN_ENV_FILE} -f ${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
rg -q -Fx "${compose_prefix} up --detach cameras" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} stop cameras" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} restart cameras" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} logs --tail=200 --follow cameras" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} down" "${CALLS_FILE}"
