# LEAP Hand Controller

> **Upstream projects:**
>
> - [IRVLUTD/leap_inhand](https://github.com/IRVLUTD/leap_inhand) -- ROS packages for LEAP Hand
> - [leap-hand/LEAP_Hand_API](https://github.com/leap-hand/LEAP_Hand_API) -- Original LEAP Hand API
> - [real-stanford/minimalist_compliance_control](https://github.com/real-stanford/minimalist_compliance_control) -- MCC compliance controller

Multiple control modes for the LEAP Hand v1 (16 DOF, XC330-M288-T), packaged in Docker with ROS Noetic + Python 3.10 + MCC.

## Control Modes

- **MCC Compliance** -- Cartesian admittance via [MCC](https://arxiv.org/abs/2603.00913). Wrench estimation, anisotropic stiffness, mink IK, C++ Dynamixel backend. For contact-rich manipulation.
- **Mode 5 Rigid** -- Firmware position-current control (kP/kD/curr_lim). For stiff position tracking.
- **Gravity Comp** -- Pinocchio-based gravity compensation. For passive compliance.

## Structure

```text
leap_hand_controller/
├── Dockerfile                  # Ubuntu 20.04 + Python 3.10 + ROS Noetic + MCC
├── docker-compose.yml
├── docker/
│   └── entrypoint.sh           # Sources ROS, sets USB latency to 1ms
├── vendor/
│   └── mcc/                    # MCC submodule (git submodule)
├── leap_hand/
│   ├── controllers/
│   │   ├── mcc_compliance/
│   │   │   └── leaphand_compliance_node.py
│   │   ├── mode5_rigid/
│   │   │   └── leaphand_node.py
│   │   └── gravity_comp/
│   │       └── leap_gravity_comp.py
│   ├── utils/
│   │   ├── set_baudrate.py
│   │   └── leap_hand_utils/
│   │       ├── dynamixel_client.py
│   │       └── leap_hand_utils.py
│   ├── launch/
│   │   ├── leap.launch               # Rigid (Mode 5) controller
│   │   ├── leap_compliance.launch    # MCC compliance controller
│   │   └── test_compliance.launch    # Controller + test trajectory
│   ├── test/
│   │   └── test_trajectory.py        # Trajectory pattern publisher
│   ├── srv/                    # ROS service definitions
│   ├── CMakeLists.txt
│   └── package.xml
├── leap_description/           # URDF models and meshes
│   ├── robots/
│   │   ├── leap_right.urdf
│   │   └── leap_left.urdf
│   └── meshes/
├── scripts/
│   └── launch.sh
└── LICENSE
```

## Quick Start

### Prerequisites

- Docker and Docker Compose
- LEAP Hand connected via USB
- Motors at **2 Mbaud** (see [Baudrate Setup](#baudrate-setup))

### Clone

```bash
git clone --recurse-submodules <your-repo-url>
cd leap_hand_controller
```

### USB Port Setup

```bash
# Find your device serial number
udevadm info -a /dev/ttyUSB0 | grep '{serial}'

# Create udev rule
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="YOUR_SERIAL", SYMLINK+="ttyLEAP_RIGHT"' \
  | sudo tee /etc/udev/rules.d/99-leap-hand.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Build and Run

```bash
docker compose build
docker compose run --rm leap_compliance bash
```

### Launch Controllers

Inside the container:

```bash
# MCC Compliance controller
roslaunch leap_hand leap_compliance.launch hand:=right

# Rigid (Mode 5) controller
roslaunch leap_hand leap.launch hand:=right
```

Both launch files start the controller node, load the URDF, publish TF via `robot_state_publisher`, and optionally open RViz (`show_rviz:=true`).

Common arguments:

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `hand` | `right` | `right` or `left` |
| `ns` | `/` | ROS namespace |
| `cmd_topic` | `cmd_leap` | Joint command topic |
| `state_topic` | `state` | Joint state topic |
| `show_rviz` | `false` | Launch RViz |

Compliance-specific arguments:

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `fw_kP` | `900` | Firmware position P gain |
| `kp_pos` | `100.0` | Positional stiffness |
| `kp_rot` | `10.0` | Rotational stiffness |

### Test Trajectories

Built-in test patterns to exercise the hand:

```bash
# All-in-one (launches controller + test trajectory)
roslaunch leap_hand test_compliance.launch hand:=right pattern:=wave cycles:=3

# Or run the test separately against a running controller
rosrun leap_hand test_trajectory.py --pattern open_close --duration 4.0 --cycles 5
```

Available patterns: `wave`, `open_close`, `pinch`, `spread`.

For standalone MCC without ROS:

```bash
cd /opt/mcc && python3 -m policy.run_policy --robot leap --sim real --policy compliance
```

## MCC Compliance

Uses [MCC](https://github.com/real-stanford/minimalist_compliance_control) end-to-end:

```text
Motor current
  → Torque estimation (Kt=0.333, gain_backdrive=5.0)
  → MuJoCo wrench estimation (Jacobian + qfrc_bias)
  → Cartesian admittance (Kp, Kd, mass per fingertip)
  → Mink IK (position-only, daqp solver)
  → Goal Position → Firmware PID (Mode 4, kP=900)
```

The ROS compliance node (`leaphand_compliance_node.py`) wraps MCC's `RealWorldDynamixel` + `CompliancePolicy` with standard ROS topics:

- Subscribes to `cmd_leap` (`sensor_msgs/JointState`) -- 16 joint positions
- Publishes `state` (`sensor_msgs/JointState`) -- position, velocity, effort (current mA)

Per-fingertip control via MCC's command matrix: position target, orientation target, stiffness (Kp 3x3), damping (Kd 3x3), force command.

## Baudrate Setup

MCC's C++ backend uses 2 Mbaud. LEAP Hand ships at 4 Mbaud. Change once:

```bash
# Inside Docker container
python3 /app/utils/set_baudrate.py --from 4000000 --to 2000000
```

Persists across power cycles.

## Joint Mapping

| Finger | MCP Side | MCP Forward | PIP | DIP |
| ------ | -------- | ----------- | --- | --- |
| Index  | 0        | 1           | 2   | 3   |
| Middle | 4        | 5           | 6   | 7   |
| Ring   | 8        | 9           | 10  | 11  |
| Thumb  | 12       | 13          | 14  | 15  |

MuJoCo swaps MCP Forward/Side (joints 0/1, 4/5, 8/9). MCC handles this mapping internally.

## Acknowledgements

- [real-stanford/minimalist_compliance_control](https://github.com/real-stanford/minimalist_compliance_control) -- MCC by Haochen Shi and Songbo Hu
- [IRVLUTD/leap_inhand](https://github.com/IRVLUTD/leap_inhand) -- ROS packages for LEAP Hand
- [leap-hand/LEAP_Hand_API](https://github.com/leap-hand/LEAP_Hand_API) -- LEAP Hand by Shaw et al.

```bibtex
@article{shaw2023leaphand,
  title={LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning},
  author={Shaw, Kenneth and Agarwal, Ananye and Pathak, Deepak},
  journal={Robotics: Science and Systems (RSS)},
  year={2023}
}
```

```bibtex
@misc{shi2026minimalist,
  title = {Minimalist {{Compliance Control}}},
  author = {Shi, Haochen and Hu, Songbo and Hou, Yifan and Wang, Weizhuo and Liu, Karen and Song, Shuran},
  year = 2026,
  month = mar,
  number = {arXiv:2603.00913},
  eprint = {2603.00913},
  primaryclass = {cs},
  publisher = {arXiv},
  doi = {10.48550/arXiv.2603.00913},
  urldate = {2026-03-03},
  archiveprefix = {arXiv},
  keywords = {Computer Science - Robotics}
}
```

## License

MIT License. See [LICENSE](LICENSE).
