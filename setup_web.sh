#!/usr/bin/env bash
set -euo pipefail
MANAGER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MANAGER_VENV="${MANAGER_DIR}/.web-venv"
if command -v uv >/dev/null 2>&1; then
  if [[ ! -x "${MANAGER_VENV}/bin/python" ]]; then
    uv venv --python /usr/bin/python3 --system-site-packages "${MANAGER_VENV}"
  fi
  uv pip install --python "${MANAGER_VENV}/bin/python" -r "${MANAGER_DIR}/requirements-web.txt"
else
  if [[ ! -x "${MANAGER_VENV}/bin/python" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "${MANAGER_VENV}"
  fi
  "${MANAGER_VENV}/bin/python" -m pip install -r "${MANAGER_DIR}/requirements-web.txt"
fi
