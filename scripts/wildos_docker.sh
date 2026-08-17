#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CALLER_DIR="${PWD}"

usage() {
  cat <<'EOF'
Usage: wildos_docker.sh <x86|orin> <action> [options] [service...]

Actions:
  init       Create the platform environment file from its template
  config     Validate the Compose configuration
  build      Build one or all services
  update     Rebuild with updated base images and recreate services
  start      Start one or all services in the background
  stop       Stop one or all services without removing containers
  down       Stop and remove all platform containers, preserving volumes
  restart    Restart one or all running services
  logs       Show the latest 200 log lines, add -f to follow
  status     Show platform container status

Options:
  --env-file PATH  Override the platform environment file
  -h, --help        Show this help

Examples:
  scripts/wildos_docker.sh x86 update
  scripts/wildos_docker.sh orin start cameras
  scripts/wildos_docker.sh orin logs -f wildos
EOF
}

fail() {
  echo "$1" >&2
  exit 1
}

[[ $# -ge 2 ]] || {
  usage >&2
  exit 1
}

PLATFORM="$1"
ACTION="$2"
shift 2

case "${PLATFORM}" in
  x86)
    ENV_FILE="${REPO_ROOT}/.env.x86_64.lidar-dlio"
    ENV_TEMPLATE="${REPO_ROOT}/.env.x86_64.lidar-dlio.example"
    COMPOSE_FILE="${REPO_ROOT}/compose.x86_64.lidar-dlio.yaml"
    ;;
  orin)
    ENV_FILE="${REPO_ROOT}/.env.orin.wildos-cameras"
    ENV_TEMPLATE="${REPO_ROOT}/.env.orin.wildos-cameras.example"
    COMPOSE_FILE="${REPO_ROOT}/compose.orin.wildos-cameras.yaml"
    ;;
  *)
    fail "Unsupported platform: ${PLATFORM}, use x86 or orin"
    ;;
esac

if [[ "${1:-}" == "--env-file" ]]; then
  [[ $# -ge 2 ]] || fail "Missing path for --env-file"
  ENV_FILE="$2"
  shift 2
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "${ENV_FILE}" != /* ]]; then
  ENV_FILE="${CALLER_DIR}/${ENV_FILE}"
fi

if [[ "${ACTION}" == "init" ]]; then
  [[ $# -eq 0 ]] || fail "init does not accept service names"
  if [[ -f "${ENV_FILE}" ]]; then
    echo "Environment file already exists: ${ENV_FILE}"
  else
    cp "${ENV_TEMPLATE}" "${ENV_FILE}"
    chmod 0600 "${ENV_FILE}"
    echo "Created environment file: ${ENV_FILE}"
  fi
  exit 0
fi

command -v docker >/dev/null 2>&1 || fail "Docker is not installed or not in PATH"
[[ -f "${ENV_FILE}" ]] || fail "Missing environment file: ${ENV_FILE}, run scripts/wildos_docker.sh ${PLATFORM} init"

cd "${REPO_ROOT}"
compose=(docker compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}")
"${compose[@]}" config --quiet

case "${ACTION}" in
  config)
    [[ $# -eq 0 ]] || fail "config does not accept service names"
    ;;
  build)
    "${compose[@]}" build "$@"
    ;;
  update)
    "${compose[@]}" build --pull "$@"
    "${compose[@]}" up --detach --force-recreate "$@"
    ;;
  start|up)
    "${compose[@]}" up --detach "$@"
    ;;
  stop)
    "${compose[@]}" stop "$@"
    ;;
  down)
    [[ $# -eq 0 ]] || fail "down does not accept service names"
    "${compose[@]}" down
    ;;
  restart)
    "${compose[@]}" restart "$@"
    ;;
  logs)
    if [[ "${1:-}" == "-f" || "${1:-}" == "--follow" ]]; then
      shift
      "${compose[@]}" logs --tail=200 --follow "$@"
    else
      "${compose[@]}" logs --tail=200 "$@"
    fi
    ;;
  status|ps)
    [[ $# -eq 0 ]] || fail "status does not accept service names"
    "${compose[@]}" ps
    ;;
  *)
    fail "Unsupported action: ${ACTION}"
    ;;
esac
