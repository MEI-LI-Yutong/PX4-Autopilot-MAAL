FROM raiots/maal_px4_simulation:latest

SHELL ["/bin/bash", "-lc"]

# Install uv and prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/* \
    && curl -LsSf https://astral.sh/uv/install.sh | sh \
    && install -m 0755 /root/.local/bin/uv /usr/local/bin/uv \
    && install -m 0755 /root/.local/bin/uvx /usr/local/bin/uvx || true

ENV PATH=/root/.local/bin:/root/.cargo/bin:$PATH

WORKDIR /PX4-Autopilot

# Bake repository into image for reproducible runs
COPY . /PX4-Autopilot

# Pre-build base target to reduce startup time (safe to re-run at container start)
ARG BUILD_TARGET=px4_sitl_default
RUN make -j"$(nproc)" ${BUILD_TARGET}

# Defaults, can be overridden at `docker run -e VAR=...`
ENV TASKS_JSON=tasks/beaufort_levels_tests.json \
    HEADLESS=0 \
    BUILD_TARGET=${BUILD_TARGET} \
    WANDB_ENTITY=MAALab \
    WANDB_PROJECT=px4_gust_eval \
    RUN_PLOTS=1 \
    UPLOAD_LOG_DATA=1

# Expose common MAVLink UDP ports (map with -p or use --network host)
EXPOSE 18570/udp 14540/udp

# Single entrypoint: build, then run task
ENTRYPOINT ["bash", "-lc", "set -euo pipefail; \
  echo '[entry] PX4 root:' $(pwd); \
  echo '[entry] BUILD_TARGET='\"${BUILD_TARGET}\"; \
  echo '[entry] TASKS_JSON='\"${TASKS_JSON}\"; \
  if [[ \"${HEADLESS}\" == \"1\" ]]; then export GZ_SIM_HEADLESS=1 QT_QPA_PLATFORM=offscreen; echo '[entry] Headless mode ON'; else echo '[entry] GUI mode'; fi; \
  make -j\"$(nproc)\" \"${BUILD_TARGET}\"; \
  cd Tools/px4_gust_eval; \
  TASKS_ARG=\"${TASKS_JSON}\"; \
  if [[ ! -f \"${TASKS_ARG}\" ]]; then \
    if [[ -f \"./${TASKS_JSON}\" ]]; then TASKS_ARG=\"./${TASKS_JSON}\"; \
    elif [[ -f \"${TASKS_JSON##*Tools/px4_gust_eval/}\" ]]; then TASKS_ARG=\"${TASKS_JSON##*Tools/px4_gust_eval/}\"; \
    elif [[ -f \"tasks/${TASKS_JSON##*/}\" ]]; then TASKS_ARG=\"tasks/${TASKS_JSON##*/}\"; fi; \
  fi; \
  echo '[entry] Running: uv run main.py' \"${TASKS_ARG}\" '--verbose'; \
  uv run --with wandb main.py \"${TASKS_ARG}\" --verbose; \
  "]
