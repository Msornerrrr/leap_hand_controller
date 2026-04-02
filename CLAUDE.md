# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

ROS Noetic controller package for the LEAP Hand v1 (16-DOF, XC330-M288-T), running in Docker with Python 3.10. Three control modes: MCC compliance (Cartesian admittance), Mode 5 rigid (firmware PID), and gravity compensation (Pinocchio).

The core compliance node (`leap_hand/controllers/mcc_compliance/leaphand_compliance_node.py`) wraps MCC's `RealWorldDynamixel` + `CompliancePolicy` with standard ROS topics. The rigid node (`controllers/mode5_rigid/leaphand_node.py`) uses the Dynamixel SDK directly.

`vendor/mcc/` is a git submodule (branch `leap-right-hand`) of [minimalist_compliance_control](https://github.com/real-stanford/minimalist_compliance_control). Treat it as external code — modify only when integration requires it.

## Build and Run

All development happens inside Docker. Source is COPY'd into the image at build time (not volume-mounted by default). **Rebuild after code changes** unless using `docker compose run` which mounts `leap_hand/`, `leap_description/`, and `vendor/mcc/` as volumes.

```bash
# Build
docker compose build leap_compliance

# Shell with live-mounted source (edits take effect without rebuild)
docker compose run --rm leap_compliance bash

# Inside container
roslaunch leap_hand leap_compliance.launch hand:=right    # MCC compliance
roslaunch leap_hand leap.launch hand:=right               # Rigid mode 5
roslaunch leap_hand test_compliance.launch pattern:=wave cycles:=3  # Controller + test trajectory

# Test trajectory against a running controller
rosrun leap_hand test_trajectory.py --pattern open_close --duration 4.0 --cycles 5

# Syntax check (no test suite exists)
python3 -m compileall -q leap_hand

# Standalone MCC (no ROS)
cd /opt/mcc && python3 -m policy.run_policy --robot leap --sim real --policy compliance
```

The Dockerfile builds from the parent directory context (`docker build -f leap_hand_controller/Dockerfile -t leap-compliance-mcc:latest .`). The catkin build uses: `catkin_make -DPYTHON_EXECUTABLE=/usr/local/bin/python3.10 -DCMAKE_POLICY_VERSION_MINIMUM=3.5`.

## Key Architecture

### MCC Compliance Pipeline

```
Motor current → Torque (Kt=0.333, gain_backdrive=5.0)
  → MuJoCo wrench estimation (Jacobian + qfrc_bias)
  → Cartesian admittance (Kp, Kd per fingertip)
  → Mink IK (position-only, daqp solver)
  → Goal Position → Firmware PID (Mode 4, kP=900)
```

### Threading Model

Both controller nodes are dual-threaded:
- **ROS spinner thread**: subscriber callbacks (`_on_cmd`, `_on_cartesian_cmd`, `_on_gravity`) queue commands under `threading.Lock()`
- **Control loop thread**: runs at fixed rate (50 Hz compliance, 80 Hz rigid), applies pending commands via `_apply_pending_commands()`, reads hardware, computes policy, writes targets

The compliance node accesses `wrench_sim.data` (MuJoCo) only from the control loop thread. State publishing happens in ROS timer callbacks, reading shared state under lock.

### Command Modes (Compliance Node)

- **Joint mode** (default, `cmd_mode=joint`): subscribes to `cmd_leap` (JointState, 16 positions) → FK to fingertip targets
- **Cartesian mode** (`cmd_mode=cartesian`): subscribes to `cmd_leap_cartesian` (PoseArray, 4 fingertip poses) → direct IK targets

### Motor Ordering Convention

LEAP hand Dynamixel IDs per finger: `[rot(MCP Side), mcp(MCP Forward), pip, dip]`. Motor ID 0=rot, 1=mcp for each finger. Index 0-3, Middle 4-7, Ring 8-11, Thumb 12-15.

MCC YAML configs (`robot.yml`, `motors_right.yml`) and MuJoCo XML actuators must match this order (rot before mcp). `RealWorldDynamixel.get_observation()` sorts by hardware Dynamixel ID via `motor_sort_idx`. `get_qpos()` uses `_motor_to_dof_mat` built from YAML key order. **Mismatch causes wrong MuJoCo FK and incorrect wrench visualization.**

### ROS Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `cmd_leap` | JointState | Sub | 16 joint position commands |
| `cmd_leap_cartesian` | PoseArray | Sub | 4 fingertip Cartesian targets (cartesian cmd_mode) |
| `mcc_gravity` | Vector3Stamped | Sub | Gravity vector in MuJoCo world frame |
| `state` | JointState | Pub | Position, velocity, effort (current mA) |
| `mcc_net_wrench` | WrenchStamped | Pub | Aggregated net wrench at palm origin |
| `mcc_debug/wrench_markers` | MarkerArray | Pub | Per-fingertip forces (red), net force (green), net torque (cyan), gravity (blue) |

### Gravity

MCC's `qfrc_bias` includes gravity torques. The node starts with **zero gravity** and expects real-time updates on `mcc_gravity` topic. Without it, finger weight appears as phantom external force. For a moving base (arm+hand), gravity direction changes with hand orientation.

```bash
rostopic pub /mcc_gravity geometry_msgs/Vector3Stamped \
  "{header: {frame_id: ''}, vector: {x: 0, y: 0, z: -9.81}}" -r 10
```

## Configuration

- **MCC gin config**: `vendor/mcc/config/leap.gin` — site names, joint mappings, IK params, motor config paths
- **Motor configs**: `vendor/mcc/descriptions/leap_hand/robot.yml`, `motors_right.yml`
- **MuJoCo models**: `vendor/mcc/descriptions/leap_hand/*.xml`
- **ROS launch**: `leap_hand/launch/leap_compliance.launch` — all runtime params

### Key Compliance Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `kp_pos` | 100.0 | Cartesian position stiffness (lower = more compliant) |
| `kp_rot` | 10.0 | Cartesian rotation stiffness |
| `fw_kP` | 900 | Dynamixel firmware P gain (lower = softer motor) |

### Key Rigid Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `kP` | 800.0 | Firmware position P gain |
| `kI` | 0.0 | Firmware I gain |
| `kD` | 200.0 | Firmware D gain |
| `curr_lim` | 550.0 | Current limit (mA) |

## Hardware Notes

- Motors must be set to **2 Mbaud** (factory is 4 Mbaud): `python3 /catkin_ws/src/leap_hand/utils/set_baudrate.py --from 4000000 --to 2000000`
- USB latency is set to 1ms by `docker/entrypoint.sh` — critical for Dynamixel timing
- USB ports should use udev symlinks (`/dev/ttyLEAP_RIGHT`, `/dev/ttyLEAP_LEFT`)
- Motor position offset: hardware uses π offset (LEAP convention) vs. URDF [0, 2π]

## Perception Integration

This repo is used as a git submodule in the `perception` repo at `src/controller/ros_pkgs/leap_hand_controller/`. Changes should be committed here first, then pulled in perception. The perception repo's `LeapHandComplianceRightConfig` generates the roslaunch command.

## Coding Conventions

- Python 3.10, 4-space indentation, `snake_case`
- Commit style: short imperative subjects ("Add X", "Fix Y")
- Keep ROS topic and parameter names stable unless the interface change is intentional
- Prefer small, localized changes over broad refactors, especially around hardware I/O and `vendor/mcc/`
