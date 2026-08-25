#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(mktemp -d)"
CALLS_FILE="${TEST_DIR}/calls"

cleanup() {
  rm -rf "${TEST_DIR}"
}
trap cleanup EXIT

for command in docker ssh sleep; do
  sed "s/@COMMAND@/${command}/" > "${TEST_DIR}/${command}" <<'EOF'
#!/usr/bin/env bash
printf '@COMMAND@ %s\n' "$*" >> "${CALLS_FILE}"
EOF
  chmod +x "${TEST_DIR}/${command}"
done

export CALLS_FILE
export PATH="${TEST_DIR}:${PATH}"

bash "${SCRIPT_DIR}/go2_search_test.sh" start "red chair"
rg -q 'ssh .*wildos_docker.sh orin start cameras wildos' "${CALLS_FILE}"
rg -q 'ssh .*go2_motion_gateway.sh start' "${CALLS_FILE}"
rg -q 'docker .*up --detach navigation' "${CALLS_FILE}"
rg -q 'docker exec -e WILDOS_SEARCH_TARGET=red chair .*object_search_target' "${CALLS_FILE}"

: > "${CALLS_FILE}"
bash "${SCRIPT_DIR}/go2_search_test.sh" stop
test "$(rg -n 'docker .*stop navigation|ssh .*go2_motion_gateway.sh stop' "${CALLS_FILE}" | wc -l)" -eq 2
test "$(rg -n 'docker .*stop navigation' "${CALLS_FILE}" | cut -d: -f1)" -lt \
  "$(rg -n 'ssh .*go2_motion_gateway.sh stop' "${CALLS_FILE}" | cut -d: -f1)"

if bash "${SCRIPT_DIR}/go2_search_test.sh" target "bad'target"; then
  echo "Unsafe target was accepted" >&2
  exit 1
fi
