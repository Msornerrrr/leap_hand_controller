# Repository Guidelines

## Project Structure & Module Organization

`leap_hand/` contains the ROS Noetic hand package: controller nodes in `controllers/`, launch files in `launch/`, service definitions in `srv/`, utilities in `utils/`, and simple motion tests in `test/`. `leap_description/` holds URDFs, RViz config, and meshes. Container setup lives in `Dockerfile`, `docker-compose.yml`, and `docker/entrypoint.sh`. `vendor/mcc/` is a vendored submodule for Minimalist Compliance Control; treat it as external code and modify it only when the integration requires it.

## Build, Test, and Development Commands

- `docker compose build leap_compliance`: build the ROS + MCC development image.
- `docker compose run --rm leap_compliance bash`: open a shell with the catkin workspace mounted for live edits.
- `roslaunch leap_hand leap.launch hand:=right`: run the rigid controller.
- `roslaunch leap_hand leap_compliance.launch hand:=right`: run the MCC compliance controller.
- `roslaunch leap_hand test_compliance.launch hand:=right pattern:=wave cycles:=3`: launch the compliance controller with the built-in trajectory publisher.
- `python3 -m compileall -q leap_hand`: fast syntax check for Python changes.

## Coding Style & Naming Conventions

Use Python 3.10, 4-space indentation, and `snake_case` for functions, params, and local variables. Keep ROS topic and parameter names stable unless the interface change is intentional and documented. Match existing file naming: controller scripts are executable Python files such as `leaphand_compliance_node.py`; launch files use descriptive names such as `leap_compliance.launch`. Prefer small, localized changes over broad refactors, especially around hardware I/O and `vendor/mcc/`.

## Testing Guidelines

This repo currently relies on smoke tests rather than a full unit-test suite. Before opening a PR, run `python3 -m compileall -q leap_hand` and at least one relevant launch test in Docker. For motion checks, use `test/test_trajectory.py` or `test_compliance.launch`. If behavior depends on hardware, note exactly what was validated, for example `leap.launch` on a right hand or `leap_compliance.launch` with RViz markers enabled.

## Commit & Pull Request Guidelines

Recent commits use short, imperative subjects such as `Add compliance launch file and test trajectory patterns` and `Remove unused mujoco_models from leap_hand package`. Follow that style: one concise subject line per logical change. Keep Docker, launch, controller, and documentation edits grouped intentionally. PRs should include a summary, affected launch commands, hardware or container validation performed, and screenshots or logs for RViz, wrench-marker, or controller-behavior changes.

## Configuration Tips

Do not commit machine-specific serial numbers, local X11 settings, or ad hoc device paths. Prefer the documented udev symlinks like `/dev/ttyLEAP_RIGHT` and keep USB, baudrate, and gravity-topic assumptions documented in `README.md`.
