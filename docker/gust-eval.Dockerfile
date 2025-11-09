FROM raiots/maal_px4_simulation:latest

# Install curl for installing uv; keep image lean
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/*

# Install uv (Python packaging/runner) non-interactively
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:/root/.cargo/bin:${PATH}"

# Default working directory with mounted PX4 tree
WORKDIR /PX4-Autopilot

# Copy helper entrypoint script
COPY docker/run_gust_eval.sh /usr/local/bin/run_gust_eval.sh
RUN chmod +x /usr/local/bin/run_gust_eval.sh

# Sensible defaults; can be overridden at `docker run` time
ENV HEADLESS=1 \
    TASKS_JSON=Tools/px4_gust_eval/tasks/dryden_boundary_layer_z_levels.json \
    BUILD_TARGET=px4_sitl_default

ENTRYPOINT ["/usr/local/bin/run_gust_eval.sh"]
