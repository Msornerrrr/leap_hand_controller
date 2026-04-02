#!/usr/bin/env python3
"""
Test trajectory publisher for LEAP Hand compliance controller.

Publishes predefined joint trajectories on cmd_leap to exercise the hand.
Run alongside the compliance (or rigid) controller launched via roslaunch.

Usage:
  rosrun leap_hand test_trajectory.py
  rosrun leap_hand test_trajectory.py --pattern wave --duration 5.0 --cycles 3
"""
import argparse
import math
import sys
import numpy as np

import rospy
from sensor_msgs.msg import JointState

# Reuse existing joint limits
sys.path.insert(0, '/catkin_ws/src/leap_hand/utils')
from leap_hand_utils.leap_hand_utils import LEAP_limits


# ── Default poses (radians, 16 joints, right hand) ─────────────────────────
# Joint order: [side, fwd, pip, dip] x index/middle/ring + [j12..j15] thumb

POSE_OPEN = np.array([
    0.0, 0.0, 0.0, 0.0,   # index
    0.0, 0.0, 0.0, 0.0,   # middle
    0.0, 0.0, 0.0, 0.0,   # ring
    0.5, 0.5, 0.3, 0.3,   # thumb
], dtype=np.float32)

POSE_CLOSED = np.array([
    0.0, 1.5, 1.5, 1.5,   # index
    0.0, 1.5, 1.5, 1.5,   # middle
    0.0, 1.5, 1.5, 1.5,   # ring
    1.0, 1.0, 1.2, 1.2,   # thumb
], dtype=np.float32)

POSE_PINCH = np.array([
    0.0, 1.2, 1.2, 1.0,   # index curled
    0.0, 0.0, 0.0, 0.0,   # middle open
    0.0, 0.0, 0.0, 0.0,   # ring open
    1.0, 1.0, 1.0, 0.8,   # thumb curled
], dtype=np.float32)

POSE_SPREAD = np.array([
    -0.4, 0.2, 0.2, 0.2,  # index spread out
     0.0, 0.2, 0.2, 0.2,  # middle
     0.4, 0.2, 0.2, 0.2,  # ring spread out
     0.3, 0.3, 0.0, 0.0,  # thumb
], dtype=np.float32)


def adapt_for_hand(q, hand):
    """Negate thumb joints 12-13 for left hand (mirrored CMC/AXL)."""
    if hand == 'left':
        q = q.copy()
        q[12] = -q[12]
        q[13] = -q[13]
    return q


def clamp_to_limits(q, hand='right'):
    """Clamp joint angles to LEAP hand limits."""
    sim_min, sim_max = LEAP_limits(hand)
    return np.clip(q, sim_min, sim_max).astype(np.float32)


def interpolate(q_start, q_end, t, duration):
    """Cosine interpolation between two poses."""
    alpha = 0.5 * (1.0 - math.cos(math.pi * min(t / duration, 1.0)))
    return q_start + alpha * (q_end - q_start)


def _alternating_pattern(pose_a, pose_b):
    """Factory for patterns that smoothly alternate between two poses."""
    def pattern(t, duration):
        phase = (t % duration) / duration
        if phase < 0.5:
            return interpolate(pose_a, pose_b, phase * 2, 1.0)
        else:
            return interpolate(pose_b, pose_a, (phase - 0.5) * 2, 1.0)
    return pattern


def pattern_wave(t, duration):
    """Sequential finger curl wave."""
    cycle_t = (t % duration) / duration
    q = POSE_OPEN.copy()

    for finger_idx in range(4):
        phase = (cycle_t - finger_idx * 0.2) % 1.0
        # Bell curve for smooth curl/uncurl per finger
        curl = math.exp(-((phase - 0.3) ** 2) / (2 * 0.04))
        base = finger_idx * 4
        if finger_idx < 3:
            q[base + 1] = curl * 1.5
            q[base + 2] = curl * 1.5
            q[base + 3] = curl * 1.5
        else:
            q[base + 0] = 0.5 + curl * 0.5
            q[base + 1] = 0.5 + curl * 0.5
            q[base + 2] = 0.3 + curl * 0.9
            q[base + 3] = 0.3 + curl * 0.9
    return q


PATTERNS = {
    'wave': pattern_wave,
    'open_close': _alternating_pattern(POSE_OPEN, POSE_CLOSED),
    'pinch': _alternating_pattern(POSE_OPEN, POSE_PINCH),
    'spread': _alternating_pattern(POSE_SPREAD, POSE_CLOSED),
}


def main():
    rospy.init_node('test_trajectory', anonymous=True)

    clean_argv = [a for a in sys.argv if not a.startswith('__')]
    parser = argparse.ArgumentParser(description='LEAP Hand test trajectory publisher')
    parser.add_argument('--pattern', choices=list(PATTERNS.keys()), default='wave',
                        help='Trajectory pattern (default: wave)')
    parser.add_argument('--duration', type=float, default=4.0,
                        help='Duration of one cycle in seconds (default: 4.0)')
    parser.add_argument('--cycles', type=int, default=3,
                        help='Number of cycles, 0 for infinite (default: 3)')
    parser.add_argument('--rate', type=float, default=30.0,
                        help='Publish rate in Hz (default: 30.0)')
    parser.add_argument('--cmd_topic', type=str, default='cmd_leap',
                        help='Command topic (default: cmd_leap)')
    parser.add_argument('--hand', type=str, default='right',
                        help='Hand type for joint limit clamping (default: right)')
    args = parser.parse_args(clean_argv[1:])

    pub = rospy.Publisher(args.cmd_topic, JointState, queue_size=1)
    rate = rospy.Rate(args.rate)
    pattern_fn = PATTERNS[args.pattern]
    joint_names = [f'joint_{i}' for i in range(16)]

    rospy.loginfo(f"Test trajectory: pattern={args.pattern}, duration={args.duration}s, "
                  f"cycles={args.cycles}, rate={args.rate}Hz, hand={args.hand}")

    total_duration = args.duration * args.cycles if args.cycles > 0 else float('inf')
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()
        if t >= total_duration:
            rospy.loginfo("Trajectory complete, sending open pose and exiting.")
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = joint_names
            msg.position = clamp_to_limits(adapt_for_hand(POSE_OPEN, args.hand), args.hand).tolist()
            pub.publish(msg)
            rospy.sleep(0.5)
            break

        q = clamp_to_limits(adapt_for_hand(pattern_fn(t, args.duration), args.hand), args.hand)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = q.tolist()
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
