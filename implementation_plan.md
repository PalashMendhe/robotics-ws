# Implementation Plan — Reproducibility & CI for robotics-ws

[Overview]

Make the ROS 2 Jazzy AMR warehouse workspace reproducible and continuously verified: layered automated tests (pure-logic unit tests, geometry-vs-world consistency tests, config validation tests), a multi-stage Docker image that *is* the reproducible environment, and GitHub Actions CI running ruff + pylint + pytest + docker build. No Gazebo in CI (user decision) — CI verifies everything that can be verified headless in minutes; the sim pipeline stays a local manual run.

Context: 4 packages — `nav_nodes` (ament_python: `mission_node.py`, `station_arm_node.py`, `broadcaster_node.py`), and `robot_description`, `arm_moveit_config`, `robotic_4dof_arm` (ament_cmake). Existing tests: `test_station_ik.py` (good offline geometry checks) and ament boilerplate where `test_copyright.py` currently fails because all four package.xml files have `TODO: License declaration`.

[Types]

No new type system (Python). Conventions only: pytest fixtures in a shared `conftest.py` (headless ROS env: unique `ROS_DOMAIN_ID`, `GZ_IP=127.0.0.1`, `use_sim_time=False`); test stubs as plain classes; image tags `robotics-ws:dev` and `robotics-ws:<git-sha>`.

[Files]

New:
- `implementation_plan.md` — this document
- `src/nav_nodes/test/conftest.py` — headless-ROS fixtures + sys.path bootstrap (centralizes the path trick currently in `test_station_ik.py`)
- `src/nav_nodes/test/test_mission_logic.py` — `_yaw_to_quat` correctness (known yaws + unit-norm), phase constants, `DESTINATIONS` sanity, dock-override parsing
- `src/nav_nodes/test/test_broadcaster_logic.py` — monotonic-stamp drop logic (duplicate / out-of-order / zero / first stamp)
- `src/nav_nodes/test/test_geometry_matches_world.py` — the drift guard: parses `large_warehouse.sdf` and asserts the hardcoded constants in `station_arm_node.py` (`ARM_X`, `ARM_Y0`, `PARCEL_X`, `PARCEL_Y0`, `DOCK_X`, `DOCK_DY`) and `mission_node.py` (`DESTINATIONS`) match within 1 mm. Why: someone moves a box in the SDF and the Python constants silently drift — this makes that a CI failure, not a mystery at sim time.
- `src/nav_nodes/test/test_configs.py` — `nav2_params.yaml` parses and has required keys; map YAML `image:` resolves and PGM dimensions match the YAML header; xacros process via the `xacro` CLI (skipped if xacro absent). Why: a malformed param/map/xacro only fails 40 s into a sim launch — this catches it in 2 s.
- `src/nav_nodes/test/test_state_machine.py` — mission transitions with mocked action/service clients: retry policy (3 nav failures → ABORT), CUE rejection retry-once, ACK 10 s timeout, WAIT_ARM `CUE_TIMEOUT` abort, phase ordering
- `Dockerfile` — multi-stage: `ros:jazzy-ros-base` → rosdep dependency layer over `src/` (cached) → `colcon build --symlink-install` → runtime stage; `dev` target with lint tools + shell extras
- `.dockerignore` — `build/ install/ log/ .git/ __pycache__/ .pytest_cache/`
- `docker-compose.yml` — `sim` and `dev` services (convenience; not used by CI)
- `.github/workflows/ci.yml` — jobs `lint` (ruff + pylint), `test` (ros:jazzy container: rosdep → `colcon build/test` for `nav_nodes` + raw pytest), `docker` (build dev target — doubles as a compile check of all packages incl. ament_cmake ones). Plus a commented, `workflow_dispatch`-gated outline of a future headless-sim smoke job.
- `ruff.toml`, `.pylintrc`, `requirements-lint.txt` — lint configs (pylint `ignored-modules` for ROS msg packages so `no-member`/`import-error` don't fire)
- `.pre-commit-config.yaml` — ruff + whitespace hooks so lint failures are caught pre-push
- `.github/dependabot.yml` — optional, Actions/docker bumps

Modified:
- All four `src/*/package.xml` — real license (Apache-2.0, fixes `ament_copyright`), real descriptions, consistent maintainer; add missing `<exec_depend>` entries (`ros_gz_sim`, `xacro`, `nav2_bringup`, `robot_state_publisher`, `joint_state_publisher`, etc.) so `rosdep install` in Docker actually pulls the full sim stack
- `broadcaster_node.py` — extract inline stamp check to pure module-level `should_accept_stamp(msg_nanos, last) -> bool`; `odom_callback` calls it (identical semantics)
- `mission_node.py` — extract dock-override `"x,y,yaw"` parsing to pure `parse_dock_override(s) -> tuple | None`
- `.gitignore` — add `.ruff_cache/`, `.venv/`, `*.egg-info/`
- `README.md` — Testing / Docker / CI section

Deleted: none.

[Functions]

New (tests): `test_yaw_to_quat_matches_expected`, `test_yaw_to_quat_unit_norm`, `test_duplicate_stamp_dropped`, `test_out_of_order_dropped`, `test_zero_stamp_dropped`, `test_first_stamp_accepted`, `test_station_constants_match_sdf`, `test_destinations_match_sdf`, `test_nav2_params_parses_and_has_keys`, `test_map_yaml_matches_pgm_dimensions`, `test_xacros_process_cleanly`, `test_retry_policy_aborts_after_three_nav_failures`, `test_cue_rejection_retries_once`, `test_ack_timeout_aborts`, `test_phase_order`.

Modified (pure refactors, no behavior change): `broadcaster_node.odom_callback` → uses `should_accept_stamp()`; `mission_node.__init__` override parsing → `parse_dock_override()`.

Removed: none.

[Classes]


[Dependencies]

- No new runtime deps for robot code.
- Dev/CI (pip, pinned): `ruff`, `pylint`.
- Docker base: `ros:jazzy-ros-base` (Ubuntu 24.04 / Python 3.12) matching the Jazzy + Gazebo Harmonic stack; Gazebo pulled via rosdep in the dependency layer.
- Actions: `actions/checkout@v4`, `docker/build-push-action@v6` (build only, no registry push yet).

[Testing]

Layered fast→slow; only fast layers in CI:
1. Pure-logic unit tests — no sim/DDS, <5 s
2. Geometry-vs-world consistency — strongest drift guard
3. Config validation — YAML/PGM/xacro
4. Existing ament tests + `test_station_ik.py` — via `colcon test --packages-select nav_nodes` inside the Docker image (so `rclpy` is present)

Local validation: `python3 -m pytest src/nav_nodes/test/ -v`; `colcon test` + `colcon test-result --verbose` (sourced ROS); `ruff check src/ && pylint src/nav_nodes/nav_nodes`; `docker build --target dev -t robotics-ws:dev .`

[Implementation Order]

1. Hygiene — package.xml licenses/descriptions/maintainers, `.gitignore` (unblocks the currently-failing copyright test)
2. Pure refactors — `should_accept_stamp()`, `parse_dock_override()`; confirm sim pipeline unchanged
3. Tests — mission logic → broadcaster → geometry-vs-world → configs → state machine, plus `conftest.py`
4. Lint config — ruff/pylint, auto-fix, iterate to clean
5. Docker — Dockerfile + `.dockerignore`; run `colcon test` inside image to prove the dependency layer is complete; compose file
6. CI — `ci.yml` + pre-commit + dependabot; verify green on GitHub
7. README + final verification pass

Rationale: hygiene fixes existing failures first; refactors precede tests targeting extracted helpers; Docker before CI since the CI test job reuses the image; CI last so it runs against already-green jobs.
