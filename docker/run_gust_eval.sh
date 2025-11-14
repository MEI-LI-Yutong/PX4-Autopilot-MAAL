#!/usr/bin/env bash
set -euo pipefail

#
# Entrypoint for automated PX4 build + gust evaluation run inside container.
#
# Environment variables (override at `docker run -e VAR=...`):
#   TASKS_JSON  - path to tasks JSON (default: Tools/px4_gust_eval/tasks/dryden_boundary_layer_z_levels.json)
#   BUILD_TARGET- make target to build first (default: px4_sitl_default)
#   HEADLESS    - 1 to run Gazebo headless (default: 1)
#

TASKS_JSON=${TASKS_JSON:-tasks/dryden_boundary_layer_z_levels.json}
BUILD_TARGET=${BUILD_TARGET:-px4_sitl_default}
HEADLESS=${HEADLESS:-1}

echo "[run_gust_eval] PX4 root: $(pwd)"
echo "[run_gust_eval] Build target: ${BUILD_TARGET}"
echo "[run_gust_eval] Tasks JSON: ${TASKS_JSON}"

# Ensure typical per-user bin dirs are on PATH (uv installer uses ~/.local/bin)
export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"

if [[ "${HEADLESS}" == "1" ]]; then
  export GZ_SIM_HEADLESS=1
  export QT_QPA_PLATFORM=offscreen
  export __GL_YIELD=USLEEP
  echo "[run_gust_eval] Running Gazebo headless"
fi

# Build PX4 SITL once so subsequent `make px4_sitl gz_...` invocations are fast
echo "[run_gust_eval] Building PX4 (${BUILD_TARGET})..."
make -j"$(nproc)" "${BUILD_TARGET}"

# Ensure uv is available
if ! command -v uv >/dev/null 2>&1; then
  echo "[run_gust_eval] uv not found; installing..."
  if command -v curl >/dev/null 2>&1; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # The installer places uv in ~/.local/bin and prints a helper env script
    # Try to source it if present to ensure PATH updates within this shell.
    if [[ -f "${HOME}/.local/bin/env" ]]; then
      # shellcheck disable=SC1090
      source "${HOME}/.local/bin/env"
    fi
    export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"
  else
    echo "[run_gust_eval] curl not available; cannot install uv automatically."
    echo "Please install uv in the image or provide it via PATH."
    exit 1
  fi
fi

# Final sanity check
if ! command -v uv >/dev/null 2>&1; then
  echo "[run_gust_eval] uv still not found on PATH after install. PATH=${PATH}"
  exit 1
fi

cd Tools/px4_gust_eval

# Normalize tasks path to be relative to Tools/px4_gust_eval when possible
TASKS_ARG="${TASKS_JSON}"
if [[ ! -f "${TASKS_ARG}" ]]; then
  if [[ -f "./${TASKS_ARG}" ]]; then
    TASKS_ARG="./${TASKS_ARG}"
  elif [[ -f "${TASKS_JSON##*Tools/px4_gust_eval/}" ]]; then
    TASKS_ARG="${TASKS_JSON##*Tools/px4_gust_eval/}"
  fi
fi

echo "[run_gust_eval] Running gust evaluation..."
echo "[run_gust_eval] Command: uv run main.py ${TASKS_ARG} --verbose"
uv run main.py "${TASKS_ARG}" --verbose
