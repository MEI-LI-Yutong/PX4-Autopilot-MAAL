#!/usr/bin/env bash
set -euo pipefail

# Simple wrapper to run the gust evaluation inside the prebuilt base image
# raiots/maal_px4_simulation:latest. It mounts the current repo at /PX4-Autopilot
# in the container, builds PX4 once, and runs the configured task set.

IMAGE=${IMAGE:-raiots/maal_px4_simulation:latest}
TASKS_JSON=${1:-Tools/px4_gust_eval/tasks/dryden_boundary_layer_z_levels.json}
BUILD_TARGET=${BUILD_TARGET:-px4_sitl_default}
HEADLESS=${HEADLESS:-1}
RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE:-ogre}

echo "[run_gust_eval_container] Using image: ${IMAGE}"
echo "[run_gust_eval_container] Tasks JSON: ${TASKS_JSON}"
echo "[run_gust_eval_container] Headless: ${HEADLESS}, Render engine: ${RENDER_ENGINE}"

docker run --rm -it \
  --privileged \
  --network host \
  --ipc=host \
  -w /PX4-Autopilot \
  -v "$(pwd)":/PX4-Autopilot:rw \
  -v /dev:/dev \
  -e TASKS_JSON="${TASKS_JSON}" \
  -e BUILD_TARGET="${BUILD_TARGET}" \
  -e HEADLESS="${HEADLESS}" \
  -e PX4_GZ_SIM_RENDER_ENGINE="${RENDER_ENGINE}" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  "${IMAGE}" \
  bash -lc 'bash /PX4-Autopilot/docker/run_gust_eval.sh'
