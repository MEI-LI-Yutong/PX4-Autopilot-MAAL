# Repository Guidelines

## Project Structure & Module Organization
- `src/` core flight stack: `modules/`, `drivers/`, `lib/` (e.g., `src/modules/<module>`).
- `boards/` board configs and build targets; `platforms/` OS layers (NuttX/POSIX).
- `ROMFS/` boot scripts and params; `msg/` uORB message definitions.
- `test/` unit/integration tests (GTest, MAVSDK, ROS rostest); `integrationtests/` helpers.
- `Tools/` developer utilities; `docs/` and `Documentation/` reference.
- `build/` generated artifacts (do not commit); `logs/` local logs.

## Build, Test, and Development Commands
- Build SITL (default): `make px4_sitl_default`
- Example NuttX target: `make px4_fmu-v5_default`
- Run SITL with Gazebo Classic: `make px4_sitl_default sitl_gazebo-classic`
- Unit/SITL tests: `make tests` (optional `TESTFILTER=<regex>`)
- CTest from build dir: `cd build/px4_sitl_test && ctest -j8`
- Quick CI-like check: `make quick_check` (build, tests, style)
- Formatting check/fix: `make check_format` / `make format`
- Static analysis: `make clang-tidy`
- Discover targets: `make list_config_targets`

## Coding Style & Naming Conventions
- C/C++: tabs for indentation (tab width 8), max line length 120 (`.editorconfig`).
- Prefer existing patterns: snake_case file names, CamelCase types; keep headers in `include/` when present.
- Use `.clang-tidy` rules (warnings as errors). Run `make check_format` before pushing.
- YAML/param files: 2-space indent; validate with `Tools/validate_yaml.py`.

## Testing Guidelines
- Frameworks: GTest/CTest for units; MAVSDK and ROS rostest for integration; SITL for behavior.
- Local: `make tests` or `ctest -R <pattern>` in `build/px4_sitl_test`.
- Coverage: `make tests_coverage` (or `tests_integration_coverage`).
- Include SITL run logs or flight logs when relevant (upload to Flight Review and link in PR).

## Commit & Pull Request Guidelines
- Commit messages: clear imperative subject + rationale; link issues (`Fixes #123`).
- One logical change per commit; keep diffs focused; update docs if behavior changes.
- PRs: describe change, rationale, test evidence (SITL logs, steps to reproduce), and affected targets. Add screenshots for UI/tools.
- Ensure submodules are up to date: `git submodule update --init --recursive`.

## Security & Configuration Tips
- Don’t commit secrets or generated artifacts (`build/`, firmware binaries).
- Use `make check` before PRs to catch style and common build issues.
- For hardware builds, verify board target locally and include upload steps if applicable.
