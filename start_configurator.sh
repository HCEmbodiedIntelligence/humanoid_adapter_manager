#!/usr/bin/env bash
set -euo pipefail
CONFIGURATOR_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/home/maple/.nvm/versions/node/v20.20.2/bin:/usr/bin:${PATH}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f "${CONFIGURATOR_DIR}/../../install/setup.bash" ]]; then
  set +u
  source "${CONFIGURATOR_DIR}/../../install/setup.bash"
  set -u
fi
export PYTHONPATH="${CONFIGURATOR_DIR}/python:${CONFIGURATOR_DIR}/../hc_teleop_recv${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -z "${HUMANOID_WEB_PYTHON:-}" ]]; then
  if [[ ! -x "${CONFIGURATOR_DIR}/.web-venv/bin/python" ]]; then
    "${CONFIGURATOR_DIR}/setup_web.sh"
  fi
  HUMANOID_WEB_PYTHON="${CONFIGURATOR_DIR}/.web-venv/bin/python"
fi
exec "${HUMANOID_WEB_PYTHON}" "${CONFIGURATOR_DIR}/scripts/configurator_launcher.py" "$@"
