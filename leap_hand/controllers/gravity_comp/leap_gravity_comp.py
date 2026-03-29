#!/usr/bin/env python3
"""
LEAP Hand gravity compensation using Pinocchio.

Provides:
- Gravity torque computation given joint angles and gravity vector
- 3D skeleton visualization with gravity vector and torque arrows
- Reusable module for integration into controllers

Usage as standalone:
    python3 leap_gravity_comp.py --hand right
    python3 leap_gravity_comp.py --hand right --gravity 0 0 -9.81
    python3 leap_gravity_comp.py --hand right --interactive
"""
import os
import sys
import numpy as np

import pinocchio as pin

# === Paths ===
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.join(SCRIPT_DIR, '..', '..', 'leap_description', 'robots')
URDF_PATHS = {
    'right': os.path.join(URDF_DIR, 'leap_right.urdf'),
    'left': os.path.join(URDF_DIR, 'leap_left.urdf'),
}

# === Constants ===
# Torque constant for XC330-M288-T (output, after 288:1 gearbox)
# Gello's empirically calibrated value: 1158.73 mA/Nm → Kt = 1/1.15873 = 0.863 Nm/A
# MCC uses 0.333 Nm/A (likely motor-side Kt, not accounting for gearbox efficiency)
KT = 1000.0 / 1158.73  # 0.863 Nm/A — from Gello's calibrated XC330_T288_T constant

FINGER_NAMES = ['Index', 'Middle', 'Ring', 'Thumb']
JOINT_NAMES = ['MCP Side', 'MCP Fwd', 'PIP', 'DIP']
MOTOR_DESC = [
    "Index MCP Side", "Index MCP Fwd", "Index PIP", "Index DIP",
    "Middle MCP Side", "Middle MCP Fwd", "Middle PIP", "Middle DIP",
    "Ring MCP Side", "Ring MCP Fwd", "Ring PIP", "Ring DIP",
    "Thumb MCP Side", "Thumb MCP Fwd", "Thumb PIP", "Thumb DIP",
]

# URDF joint names
URDF_JOINT_NAMES = [f'joint_{i}' for i in range(16)]

# Finger colors for visualization
FINGER_COLORS = {
    'Index': '#e41a1c',   # red
    'Middle': '#377eb8',  # blue
    'Ring': '#4daf4a',    # green
    'Thumb': '#984ea3',   # purple
}

# Which motor IDs belong to which finger
FINGER_MOTOR_IDS = {
    'Index': [0, 1, 2, 3],
    'Middle': [4, 5, 6, 7],
    'Ring': [8, 9, 10, 11],
    'Thumb': [12, 13, 14, 15],
}

# Parent-child joint relationships for drawing skeleton
# Each finger chain: palm → MCP Fwd → MCP Side → PIP → DIP
FINGER_JOINT_CHAINS = {
    'Index': [1, 0, 2, 3],     # motor IDs in kinematic chain order
    'Middle': [5, 4, 6, 7],
    'Ring': [9, 8, 10, 11],
    'Thumb': [12, 13, 14, 15],
}


class LeapGravityComp:
    """Gravity compensation for LEAP Hand using Pinocchio."""

    def __init__(self, hand='right'):
        urdf_path = URDF_PATHS[hand]
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.hand = hand

        # Discover joint mapping: motor ID <-> Pinocchio index
        self.motor_to_pin_q = {}
        self.motor_to_pin_v = {}
        self.pin_joint_to_motor = {}

        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            joint = self.model.joints[i]
            for motor_id, urdf_name in enumerate(URDF_JOINT_NAMES):
                if name == urdf_name:
                    self.motor_to_pin_q[motor_id] = joint.idx_q
                    self.motor_to_pin_v[motor_id] = joint.idx_v
                    self.pin_joint_to_motor[i] = motor_id
                    break

        # Per-joint scale factors (default 1.0, calibrate with calibrate_gravity())
        self.joint_scales = np.ones(16)

        # Friction model placeholder — not calibrated by default
        # Can be enabled later via load_friction_params() if sysid is done
        self.friction_enabled = False

        # Joint limit barriers (virtual walls at URDF limits)
        self.joint_limit_kp = 1.0   # Nm/rad
        self.joint_limit_kd = 0.05  # Nm·s/rad
        self.joint_limits_min = np.zeros(16)
        self.joint_limits_max = np.zeros(16)
        self.joint_limits_margin = 0.05  # rad safety margin
        self._load_joint_limits()

    def set_joint_scales(self, scales):
        """Set per-joint gravity compensation scale factors.

        Args:
            scales: array of 16 scale factors. 1.0 = use model as-is.
                    < 1.0 = reduce comp (friction helps more than model expects).
                    > 1.0 = increase comp (model underestimates gravity load).
        """
        self.joint_scales = np.array(scales, dtype=float)

    def _load_joint_limits(self):
        """Load joint limits from the Pinocchio model."""
        for motor_id in range(16):
            if motor_id in self.motor_to_pin_q:
                idx = self.motor_to_pin_q[motor_id]
                self.joint_limits_min[motor_id] = float(self.model.lowerPositionLimit[idx]) + self.joint_limits_margin
                self.joint_limits_max[motor_id] = float(self.model.upperPositionLimit[idx]) - self.joint_limits_margin

    def friction_compensation(self, vel):
        """Friction compensation placeholder. Returns zeros unless calibrated."""
        return np.zeros(16)

    def joint_limit_barrier(self, q_motor, vel):
        """Compute joint limit barrier torques (virtual walls at limits).

        Pushes joints back when they approach URDF limits.

        Args:
            q_motor: joint angles indexed by motor ID (16,), rad
            vel: joint velocities indexed by motor ID (16,), rad/s

        Returns:
            tau_barrier: barrier torques (16,), Nm
        """
        tau_barrier = np.zeros(16)

        exceed_max = q_motor > self.joint_limits_max
        if np.any(exceed_max):
            tau_barrier[exceed_max] = (
                -self.joint_limit_kp * (q_motor[exceed_max] - self.joint_limits_max[exceed_max])
                - self.joint_limit_kd * vel[exceed_max]
            )

        exceed_min = q_motor < self.joint_limits_min
        if np.any(exceed_min):
            tau_barrier[exceed_min] = (
                -self.joint_limit_kp * (q_motor[exceed_min] - self.joint_limits_min[exceed_min])
                - self.joint_limit_kd * vel[exceed_min]
            )

        return tau_barrier

    def set_gravity(self, gravity_vec):
        """Set the gravity vector in the palm base frame.

        Args:
            gravity_vec: [gx, gy, gz] in m/s^2, expressed in palm_lower frame.
                         e.g., [0, 0, -9.81] if palm z-axis points up.
        """
        g = np.array(gravity_vec, dtype=float)
        self.model.gravity = pin.Motion(np.array([g[0], g[1], g[2], 0, 0, 0]))

    def _motor_to_pin(self, q_motor, v_motor=None):
        """Convert motor-ordered arrays to Pinocchio ordering.

        Args:
            q_motor: joint angles indexed by motor ID (16,)
            v_motor: joint velocities indexed by motor ID (16,), optional

        Returns:
            q_pin, v_pin: arrays in Pinocchio ordering
        """
        q = pin.neutral(self.model)
        v = np.zeros(self.model.nv)
        for motor_id in range(16):
            if motor_id in self.motor_to_pin_q:
                q[self.motor_to_pin_q[motor_id]] = q_motor[motor_id]
            if v_motor is not None and motor_id in self.motor_to_pin_v:
                v[self.motor_to_pin_v[motor_id]] = v_motor[motor_id]
        return q, v

    def _pin_to_motor(self, tau_pin):
        """Convert Pinocchio-ordered torque vector to motor ordering."""
        tau_motor = np.zeros(16)
        for motor_id in range(16):
            if motor_id in self.motor_to_pin_v:
                tau_motor[motor_id] = tau_pin[self.motor_to_pin_v[motor_id]]
        return tau_motor

    def compute_gravity_torques(self, q_motor, gravity_vec=None, v_motor=None):
        """Compute gravity (+ Coriolis) compensation torques using RNEA.

        Follows Gello's approach: pin.rnea(q, qdot, zero_accel).
        At zero velocity this is pure gravity. With velocity it includes Coriolis.

        Args:
            q_motor: joint angles indexed by motor ID (16,), in radians
            gravity_vec: optional gravity vector override [gx, gy, gz]
            v_motor: joint velocities indexed by motor ID (16,), in rad/s.
                     If None, assumes zero velocity (pure gravity comp).

        Returns:
            tau_g_motor: compensation torques indexed by motor ID (16,), in Nm
        """
        if gravity_vec is not None:
            self.set_gravity(gravity_vec)

        q, v = self._motor_to_pin(q_motor, v_motor)

        # RNEA with zero acceleration = gravity + Coriolis (same as Gello)
        tau = pin.rnea(self.model, self.data, q, v, np.zeros(self.model.nv))

        tau_motor = self._pin_to_motor(tau)
        tau_motor *= self.joint_scales
        return tau_motor

    def torque_to_current(self, tau):
        """Convert torques (Nm) to motor current (mA).

        Uses Gello's calibrated mapping: 1158.73 mA/Nm for XC330-M288-T.
        """
        return tau * (1000.0 / KT)  # Nm * (mA/Nm) = mA

    # Keep old name for backward compat
    def gravity_to_current(self, tau_g):
        """Convert gravity torques (Nm) to motor current (mA)."""
        return self.torque_to_current(tau_g)

    def forward_kinematics(self, q_motor):
        """Compute forward kinematics, return joint positions in base frame.

        Args:
            q_motor: joint angles indexed by motor ID (16,)

        Returns:
            joint_positions: dict mapping motor_id → [x, y, z] in base frame
            palm_position: [x, y, z] of palm origin (always [0,0,0] for fixed base)
        """
        q = pin.neutral(self.model)
        for motor_id in range(16):
            if motor_id in self.motor_to_pin_q:
                q[self.motor_to_pin_q[motor_id]] = q_motor[motor_id]

        pin.forwardKinematics(self.model, self.data, q)

        joint_positions = {}
        for i in range(1, self.model.njoints):
            if i in self.pin_joint_to_motor:
                motor_id = self.pin_joint_to_motor[i]
                pos = self.data.oMi[i].translation.copy()
                joint_positions[motor_id] = pos

        palm_position = np.zeros(3)
        return joint_positions, palm_position

    def visualize(self, q_motor=None, gravity_vec=None, tau_g=None, title=None,
                  ax=None, show=True):
        """Visualize the hand skeleton with gravity vector and torque arrows.

        Args:
            q_motor: joint angles (16,). If None, uses zeros.
            gravity_vec: gravity vector in base frame. If None, uses model's current.
            tau_g: precomputed gravity torques. If None, computes them.
            title: plot title
            ax: existing matplotlib 3D axis. If None, creates new figure.
            show: whether to call plt.show()

        Returns:
            fig, ax
        """
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        if q_motor is None:
            q_motor = np.zeros(16)

        if gravity_vec is not None:
            self.set_gravity(gravity_vec)
        else:
            gravity_vec = self.model.gravity.linear.copy()

        if tau_g is None:
            tau_g = self.compute_gravity_torques(q_motor, gravity_vec)

        # Get joint positions
        joint_pos, palm_pos = self.forward_kinematics(q_motor)

        # Create figure
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
        else:
            fig = ax.get_figure()

        # Draw palm as a point
        ax.scatter(*palm_pos, s=200, c='black', marker='s', label='Palm', zorder=5)

        # Draw each finger chain
        for finger_name, chain in FINGER_JOINT_CHAINS.items():
            color = FINGER_COLORS[finger_name]

            # Draw links: palm → first joint → second → ...
            positions = [palm_pos]
            for motor_id in chain:
                if motor_id in joint_pos:
                    positions.append(joint_pos[motor_id])

            positions = np.array(positions)
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                    color=color, linewidth=2.5, label=finger_name, zorder=3)

            # Draw joint dots
            for i, motor_id in enumerate(chain):
                if motor_id in joint_pos:
                    p = joint_pos[motor_id]
                    ax.scatter(*p, s=60, c=color, zorder=4)

                    # Annotate with motor ID and torque
                    t = tau_g[motor_id]
                    i_ma = t / KT * 1000
                    if abs(i_ma) > 1.0:  # Only label significant torques
                        ax.text(p[0], p[1], p[2] + 0.003,
                                f'M{motor_id}\n{i_ma:+.0f}mA',
                                fontsize=7, ha='center', color=color)

        # Draw gravity vector as a thick arrow from palm
        g_norm = np.linalg.norm(gravity_vec)
        if g_norm > 0:
            g_dir = np.array(gravity_vec) / g_norm
            arrow_scale = 0.05  # 5cm arrow for full gravity
            arrow_end = palm_pos + g_dir * arrow_scale
            ax.quiver(palm_pos[0], palm_pos[1], palm_pos[2],
                      g_dir[0] * arrow_scale, g_dir[1] * arrow_scale, g_dir[2] * arrow_scale,
                      color='red', linewidth=3, arrow_length_ratio=0.2,
                      label=f'Gravity [{gravity_vec[0]:.1f}, {gravity_vec[1]:.1f}, {gravity_vec[2]:.1f}]')

        # Draw world frame axes at palm
        axis_len = 0.03
        for axis_idx, (axis_color, axis_label) in enumerate(
                [('red', 'X'), ('green', 'Y'), ('blue', 'Z')]):
            direction = np.zeros(3)
            direction[axis_idx] = axis_len
            ax.quiver(palm_pos[0], palm_pos[1], palm_pos[2],
                      direction[0], direction[1], direction[2],
                      color=axis_color, linewidth=1.5, arrow_length_ratio=0.15, alpha=0.5)
            end = palm_pos + direction
            ax.text(end[0], end[1], end[2], axis_label,
                    fontsize=8, color=axis_color, alpha=0.7)

        # Formatting
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

        if title is None:
            title = f'LEAP Hand ({self.hand}) — g=[{gravity_vec[0]:.1f}, {gravity_vec[1]:.1f}, {gravity_vec[2]:.1f}]'
        ax.set_title(title)

        # Equal aspect ratio
        all_pos = np.array([palm_pos] + [joint_pos[m] for m in joint_pos])
        center = all_pos.mean(axis=0)
        max_range = (all_pos.max(axis=0) - all_pos.min(axis=0)).max() / 2 * 1.3
        if max_range < 0.01:
            max_range = 0.05
        ax.set_xlim(center[0] - max_range, center[0] + max_range)
        ax.set_ylim(center[1] - max_range, center[1] + max_range)
        ax.set_zlim(center[2] - max_range, center[2] + max_range)

        ax.legend(loc='upper left', fontsize=9)

        if show:
            plt.tight_layout()
            plt.show()

        return fig, ax

    def visualize_multi_gravity(self, q_motor=None, gravity_vectors=None):
        """Show the hand with multiple gravity directions side by side.

        Args:
            q_motor: joint angles (16,)
            gravity_vectors: dict of {label: [gx, gy, gz]}
        """
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt

        if q_motor is None:
            q_motor = np.zeros(16)

        if gravity_vectors is None:
            gravity_vectors = {
                'z-down\n(palm up)': [0, 0, -9.81],
                'z-up\n(palm down)': [0, 0, 9.81],
                'x-down': [-9.81, 0, 0],
                'y-down': [0, -9.81, 0],
            }

        n = len(gravity_vectors)
        fig = plt.figure(figsize=(6 * n, 8))

        for idx, (label, g_vec) in enumerate(gravity_vectors.items()):
            ax = fig.add_subplot(1, n, idx + 1, projection='3d')
            tau_g = self.compute_gravity_torques(q_motor, g_vec)

            # Build torque summary for subtitle
            fwd_torques = []
            for f_idx, finger in enumerate(FINGER_NAMES):
                motor_fwd = f_idx * 4 + 1
                i_ma = tau_g[motor_fwd] / KT * 1000
                fwd_torques.append(f'{finger[0]}:{i_ma:+.0f}')
            torque_str = '  '.join(fwd_torques)

            self.visualize(q_motor, g_vec, tau_g,
                          title=f'{label}\nFwd mA: {torque_str}',
                          ax=ax, show=False)

        fig.suptitle(f'LEAP Hand Gravity Torques — Different Orientations', fontsize=14, y=1.02)
        plt.tight_layout()
        plt.show()
        return fig


# === Standalone usage ===
def main():
    import argparse
    parser = argparse.ArgumentParser(description='LEAP Hand gravity compensation visualization')
    parser.add_argument('--hand', default='right', choices=['left', 'right'])
    parser.add_argument('--gravity', type=float, nargs=3, default=[0, 0, -9.81],
                        metavar=('GX', 'GY', 'GZ'),
                        help='Gravity vector in palm base frame (default: 0 0 -9.81)')
    parser.add_argument('--q', type=float, nargs=16, default=None,
                        help='Joint angles for all 16 motors (radians)')
    parser.add_argument('--multi', action='store_true',
                        help='Show multiple gravity directions side by side')
    parser.add_argument('--interactive', action='store_true',
                        help='Interactive mode: rotate gravity with sliders')
    args = parser.parse_args()

    gc = LeapGravityComp(hand=args.hand)
    q = np.array(args.q) if args.q else np.zeros(16)

    if args.multi:
        gc.visualize_multi_gravity(q)
    elif args.interactive:
        run_interactive(gc, q)
    else:
        tau_g = gc.compute_gravity_torques(q, args.gravity)
        print("\nGravity torques:")
        for i in range(16):
            i_ma = tau_g[i] / KT * 1000
            print(f"  Motor {i:2d} ({MOTOR_DESC[i]:>18s}): {tau_g[i]:+.5f} Nm  ({i_ma:+.1f} mA)")
        gc.visualize(q, args.gravity, tau_g)


def run_interactive(gc, q_motor):
    """Interactive mode with sliders to rotate gravity direction."""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.25)

    # Sliders for roll/pitch (rotation of gravity vector)
    ax_pitch = plt.axes([0.2, 0.12, 0.6, 0.03])
    ax_roll = plt.axes([0.2, 0.06, 0.6, 0.03])
    slider_pitch = Slider(ax_pitch, 'Pitch (deg)', -180, 180, valinit=0)
    slider_roll = Slider(ax_roll, 'Roll (deg)', -180, 180, valinit=0)

    def update(val):
        pitch = np.radians(slider_pitch.val)
        roll = np.radians(slider_roll.val)

        # Rotate gravity: start with [0,0,-9.81], apply roll (around x) then pitch (around y)
        Rx = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        g = Ry @ Rx @ np.array([0, 0, -9.81])

        ax.cla()
        tau_g = gc.compute_gravity_torques(q_motor, g)
        gc.visualize(q_motor, g, tau_g,
                    title=f'Pitch={slider_pitch.val:.0f}  Roll={slider_roll.val:.0f}  '
                          f'g=[{g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f}]',
                    ax=ax, show=False)
        fig.canvas.draw_idle()

    slider_pitch.on_changed(update)
    slider_roll.on_changed(update)

    # Initial draw
    update(None)
    plt.show()


if __name__ == '__main__':
    main()
