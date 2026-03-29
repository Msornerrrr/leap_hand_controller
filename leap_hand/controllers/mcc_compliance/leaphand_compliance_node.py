#!/usr/bin/env python3
"""
LEAP Hand MCC compliance controller — ROS node.

Thin ROS wrapper around MCC's full pipeline:
  RealWorldDynamixel (C++ hardware) + CompliancePolicy (compliance stack)

Same interface as leaphand_node.py:
  - Subscribes to cmd_leap (JointState) for teleop joint commands
  - Publishes state (JointState) with position/velocity/effort(current)

Internally runs the exact same loop as:
  python3 -m policy.run_policy --robot leap --sim real --policy compliance

Usage:
  python3 leaphand_compliance_node.py
  python3 leaphand_compliance_node.py --frequency 50
"""
import os
import sys
import time
import threading
import numpy as np

import rospy
from sensor_msgs.msg import JointState

# ── MCC imports ──────────────────────────────────────────────────────────────
import gin
from minimalist_compliance_control.controller import ControllerConfig
from minimalist_compliance_control.utils import load_merged_motor_config
from real_world.real_world_dynamixel import RealWorldDynamixel
from policy.compliance import CompliancePolicy
from policy.run_policy import MotorConfigPaths


class LeapComplianceNode:
    """ROS wrapper around MCC's RealWorldDynamixel + CompliancePolicy.

    The control loop is identical to run_policy.py:
      obs = sim.get_observation()
      action = policy.step(obs, sim)
      sim.set_motor_target(action)

    The only additions:
      - cmd_leap subscriber updates policy.pose_command via FK
      - state publisher exposes pos/vel/cur as JointState
    """

    def __init__(self, frequency):
        self.frequency = frequency
        self.hand = rospy.get_param('~hand', 'right')
        self.cmd_topic = rospy.get_param('~cmd_topic', 'cmd_leap')
        self.state_topic = rospy.get_param('~state_topic', 'state')

        # ── Initialize MCC (same as run_policy.py main()) ────────────────
        mcc_root = os.environ.get('MCC_ROOT', '/opt/mcc')
        os.chdir(mcc_root)

        gin_file = 'config/leap.gin'
        gin.parse_config_file(gin_file, skip_unknown=True)

        motor_cfg_paths = MotorConfigPaths()
        default_cfg = os.path.join(mcc_root, str(motor_cfg_paths.default_config_path))
        robot_cfg = os.path.join(mcc_root, str(motor_cfg_paths.robot_config_path))
        motors_cfg = (
            os.path.join(mcc_root, str(motor_cfg_paths.motor_config_path))
            if motor_cfg_paths.motor_config_path else None
        )
        merged_config = load_merged_motor_config(default_cfg, robot_cfg, motors_cfg)

        controller_cfg = ControllerConfig()
        xml_path = os.path.join(mcc_root, str(controller_cfg.xml_path))

        # Build RealWorldDynamixel (MCC's C++ hardware backend)
        control_dt = 1.0 / frequency
        self.sim = RealWorldDynamixel(
            robot='leap',
            control_dt=control_dt,
            xml_path=xml_path,
            merged_config=merged_config,
        )
        rospy.loginfo("RealWorldDynamixel initialized")

        # Get initial observation (with retries for first read)
        init_obs = self.sim.get_observation(retries=-1)
        init_motor_pos = np.asarray(init_obs.motor_pos, dtype=np.float32)

        # Build CompliancePolicy (MCC's full compliance stack)
        # This includes the gentle prep trajectory (7 sec interpolation)
        self.policy = CompliancePolicy(
            name='compliance',
            robot='leap',
            init_motor_pos=init_motor_pos,
            start_keyboard_listener=False,
            enable_plotter=False,
            enable_force_perturbation=False,
        )
        rospy.loginfo("CompliancePolicy initialized")

        # Track start time so obs.time starts from 0
        # (policy.step uses obs.time for prep trajectory interpolation)
        self.start_time = None

        # ── ROS interface ────────────────────────────────────────────────
        self.lock = threading.Lock()
        self.joint_names = [f"joint_{i}" for i in range(16)]

        # Latest obs for publishing (in motor order as the original node does)
        self.latest_pos = np.zeros(16, dtype=np.float32)
        self.latest_vel = np.zeros(16, dtype=np.float32)
        self.latest_cur = np.zeros(16, dtype=np.float32)

        rospy.Subscriber(self.cmd_topic, JointState, self._on_cmd, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, JointState, queue_size=10)
        self.pub_timer = rospy.Timer(
            rospy.Duration(1.0 / frequency), self._publish_state)

        # Start control loop
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        rospy.loginfo(f"Compliance node running at {frequency} Hz")

    def _on_cmd(self, msg):
        """Receive joint command → convert to Cartesian target via MCC's FK."""
        q_cmd = np.array(msg.position, dtype=np.float32)

        # Convert motor-order joint command to qpos using RealWorldDynamixel's mapping
        qpos_cmd = self.sim.get_qpos(q_cmd)

        # FK: get fingertip poses from these joint angles
        with self.lock:
            self.policy.controller.sync_qpos(qpos_cmd)
            x_target = self.policy.controller.get_x_obs()
            self.policy.pose_command = x_target.copy()
            self.policy.base_pose_command = x_target.copy()

    def _control_loop(self):
        """Main loop — identical to run_policy.py's run_policy()."""
        next_tick = time.monotonic()
        dt = 1.0 / self.frequency

        while not rospy.is_shutdown():
            try:
                obs = self.sim.get_observation()

                # Zero the time so policy's prep trajectory works correctly
                if self.start_time is None:
                    self.start_time = float(obs.time)
                obs.time -= self.start_time

                # Step MCC policy (prep interpolation → alignment → compliance)
                action = self.policy.step(obs, self.sim)
                action_arr = np.asarray(action, dtype=np.float32)

                # Write to hardware
                self.sim.set_motor_target(action_arr)
                self.sim.step()
                self.sim.sync()

                # Cache state for ROS publishing
                # obs.motor_pos is in MCC's sorted motor order (= config order)
                # which matches joint_0..joint_15 for the LEAP hand
                self.latest_pos = np.asarray(obs.motor_pos, dtype=np.float32)
                self.latest_vel = np.asarray(obs.motor_vel, dtype=np.float32)
                self.latest_cur = np.asarray(
                    obs.motor_cur if obs.motor_cur is not None
                    else np.zeros(16), dtype=np.float32)

            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")

            # Rate limit (same as run_policy.py)
            next_tick += dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def _publish_state(self, _event=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.latest_pos.tolist()
        msg.velocity = self.latest_vel.tolist()
        msg.effort = self.latest_cur.tolist()
        self.state_pub.publish(msg)

    def shutdown(self):
        try:
            self.policy.close()
        except Exception:
            pass
        self.sim.close()


if __name__ == '__main__':
    rospy.init_node('leaphand_node')

    import argparse
    ros_args = [a for a in sys.argv if a.startswith('__')]
    clean_argv = [a for a in sys.argv if not a.startswith('__')]
    sys.argv = clean_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=float, default=50.0)
    args = parser.parse_args()

    node = LeapComplianceNode(args.frequency)
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
