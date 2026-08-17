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

docker() {
  printf '%s\n' "$*" >> "${CALLS_FILE}"
}
export -f docker
export CALLS_FILE

bash "${SCRIPT_DIR}/wildos_docker.sh" x86 init --env-file "${ENV_FILE}" >/dev/null
test -f "${ENV_FILE}"

bash "${SCRIPT_DIR}/wildos_docker.sh" x86 update --env-file "${ENV_FILE}" localization

compose_prefix="compose --env-file ${ENV_FILE} -f ${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
rg -q -Fx "${compose_prefix} config --quiet" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} build --pull localization" "${CALLS_FILE}"
rg -q -Fx "${compose_prefix} up --detach --force-recreate localization" "${CALLS_FILE}"

cp "${REPO_ROOT}/.env.orin.wildos-cameras.example" "${ORIN_ENV_FILE}"
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
