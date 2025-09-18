# Repository Guidelines

## Project Structure & Module Organization
- Core code in `src/`: `src/modules/` (flight apps), `src/drivers/`, `src/lib/`, `src/systemcmds/`.
- Board configs in `boards/<vendor>/<board>/`; platforms in `platforms/`.
- Messages in `msg/` (uORB). ROMFS and startup scripts in `ROMFS/`.
- Tests in `test/` (CTests/GTest, ROS/MAVSDK integration). Utilities in `Tools/`.
- Docs in `docs/`. Build artifacts in `build/` (auto‑created per target).

## Build, Test, and Development Commands
- Setup (Python tools): `pip install -r Tools/setup/requirements.txt`
- Build SITL (default): `make px4_sitl_default`
- Run SITL with Gazebo Classic: `make px4_sitl_default sitl_gazebo-classic`
- Build hardware firmware (example): `make px4_fmu-v6x_default [upload]`
- List all build targets: `make list_config_targets`
- Quick CI-like check: `make quick_check` (builds, tests, style)
- Clean builds: `make clean` (or `make distclean` to wipe and reset)

## Coding Style & Naming Conventions
- C/C++: tabs, tab width 8 (`.editorconfig`). YAML: 2 spaces.
- Format check/fix (astyle): `make check_format` / `make format`.
- Optional static checks: `make clang-tidy` (or `clang-tidy-fix`).
- Filenames lower_snake_case; classes UpperCamelCase; constants UPPER_SNAKE_CASE.

## Testing Guidelines
- Unit tests (CTest/GTest): `make tests`.
- Filter tests: `TESTFILTER=<regex> make tests`.
- Integration (SITL + ROS/MAVSDK): `make rostest` or `make tests_integration`.
- Coverage: `make tests_coverage` → report at `coverage/lcov.info`.

## Commit & Pull Request Guidelines
- Commits: imperative subject, concise body; reference issues (e.g., `Fixes #123`).
- PRs: follow `.github/PULL_REQUEST_TEMPLATE.md`; include problem, solution, linked issues, test results, and SITL/hardware logs (e.g., review.px4.io links).
- Before pushing: `make check_format && make tests` (or `make quick_check`). Ensure submodules are up to date.

## Security & Configuration Tips
- Submodules: `make submodulesupdate` (required after branch switches).
- Build config: `make updateconfig` (Kconfig sync). External modules via `EXTERNAL_MODULES_LOCATION`.
- Sanitizers (SITL): prefix with `PX4_ASAN=1`, `PX4_TSAN=1`, etc., e.g., `PX4_ASAN=1 make px4_sitl_default`.

