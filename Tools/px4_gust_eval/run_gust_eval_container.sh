#!/usr/bin/env bash
set -euo pipefail

# Simple wrapper to run the gust evaluation inside the prebuilt base image
# raiots/maal_px4_simulation:latest. It mounts the current repo at /PX4-Autopilot
# in the container, builds PX4 once, and runs the configured task set.

IMAGE=${IMAGE:-raiots/maal_px4_simulation:latest}
TASKS_JSON=${1:-Tools/px4_gust_eval/tasks/dryden_boundary_layer_z_levels.json}
BUILD_TARGET=${BUILD_TARGET:-px4_sitl_default}
HEADLESS=${HEADLESS:-1}

echo "[run_gust_eval_container] Using image: ${IMAGE}"
echo "[run_gust_eval_container] Tasks JSON: ${TASKS_JSON}"

docker run --rm -it \
  -w /PX4-Autopilot \
  -v "$(pwd)":/PX4-Autopilot:rw \
  -e TASKS_JSON="${TASKS_JSON}" \
  -e BUILD_TARGET="${BUILD_TARGET}" \
  -e HEADLESS="${HEADLESS}" \
  "${IMAGE}" \
  bash -lc 'bash /PX4-Autopilot/docker/run_gust_eval.sh'
