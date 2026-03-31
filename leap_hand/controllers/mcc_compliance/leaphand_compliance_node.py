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

import mujoco
import rospy
from geometry_msgs.msg import PoseArray, Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

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
        self.cmd_mode = rospy.get_param('~cmd_mode', 'joint')
        self.cartesian_cmd_topic = rospy.get_param(
            '~cartesian_cmd_topic', 'cmd_leap_cartesian'
        )
        self.gravity_topic = rospy.get_param('~gravity_topic', 'mcc_gravity')
        self.publish_debug_markers = bool(
            rospy.get_param('~publish_debug_markers', False)
        )
        self.debug_force_scale = float(rospy.get_param('~debug_force_scale', 0.02))
        self.debug_gravity_scale = float(
            rospy.get_param('~debug_gravity_scale', 0.01)
        )
        self.debug_frame = rospy.get_param('~debug_frame', 'palm_lower')

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
        # Default to zero gravity — real gravity comes via the gravity topic
        self.policy.controller.wrench_sim.model.opt.gravity[:] = 0.0
        rospy.loginfo("CompliancePolicy initialized (gravity zeroed, awaiting topic)")

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
        self.latest_wrenches = {}
        self.latest_site_positions = {}
        self.latest_net_wrench = np.zeros(6, dtype=np.float32)
        self.latest_gravity = np.zeros(3, dtype=np.float32)
        self.live_gravity = None
        self._pending_qpos_cmd = None
        self._pending_cartesian_cmd = None

        if self.cmd_mode == 'cartesian':
            rospy.Subscriber(
                self.cartesian_cmd_topic, PoseArray,
                self._on_cartesian_cmd, queue_size=1,
            )
            rospy.loginfo(
                "Cartesian command mode: listening on '%s' (PoseArray)",
                self.cartesian_cmd_topic,
            )
        else:
            rospy.Subscriber(
                self.cmd_topic, JointState, self._on_cmd, queue_size=1,
            )
            rospy.loginfo(
                "Joint command mode: listening on '%s' (JointState)",
                self.cmd_topic,
            )
        rospy.Subscriber(
            self.gravity_topic, Vector3Stamped, self._on_gravity, queue_size=1
        )
        rospy.loginfo("Listening for gravity on '%s'", self.gravity_topic)
        self.state_pub = rospy.Publisher(self.state_topic, JointState, queue_size=10)
        self.gravity_pub = rospy.Publisher(
            'mcc_debug/gravity', Vector3Stamped, queue_size=1
        )
        self.net_wrench_pub = rospy.Publisher(
            'mcc_net_wrench', WrenchStamped, queue_size=1
        )
        self.wrench_marker_pub = rospy.Publisher(
            'mcc_debug/wrench_markers', MarkerArray, queue_size=1
        )
        self.pub_timer = rospy.Timer(
            rospy.Duration(1.0 / frequency), self._publish_state)

        self.site_ids = dict(self.policy.controller.site_ids)
        self.palm_body_id = mujoco.mj_name2id(
            self.policy.controller.wrench_sim.model,
            mujoco.mjtObj.mjOBJ_BODY,
            'palm',
        )

        # Start control loop
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        rospy.loginfo(f"Compliance node running at {frequency} Hz")

    def _on_gravity(self, msg):
        with self.lock:
            self.live_gravity = np.array(
                [msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32
            )

    def _apply_gravity(self):
        with self.lock:
            if self.live_gravity is not None:
                gravity = self.live_gravity.copy()
            else:
                gravity = np.asarray(
                    self.policy.controller.wrench_sim.model.opt.gravity,
                    dtype=np.float32,
                ).copy()
        self.policy.controller.wrench_sim.model.opt.gravity[:] = gravity
        self.latest_gravity = gravity

    def _ordered_ros_values_from_msg(self, msg):
        if msg.name and len(msg.name) == len(msg.position):
            pos_by_name = {name: pos for name, pos in zip(msg.name, msg.position)}
            missing = [name for name in self.joint_names if name not in pos_by_name]
            if missing:
                rospy.logwarn_throttle(
                    1.0,
                    "JointState missing expected names: %s",
                    ", ".join(missing),
                )
                return None
            return np.asarray(
                [pos_by_name[name] for name in self.joint_names], dtype=np.float32
            )

        q_cmd_ros = np.asarray(msg.position, dtype=np.float32)
        if q_cmd_ros.shape[0] != len(self.joint_names):
            rospy.logwarn_throttle(
                1.0,
                f"Expected {len(self.joint_names)} commanded joints, got {q_cmd_ros.shape[0]}",
            )
            return None
        return q_cmd_ros

    def _on_cmd(self, msg):
        """Receive joint command — queue for FK in the control loop thread."""
        q_cmd_ros = self._ordered_ros_values_from_msg(msg)
        if q_cmd_ros is None:
            return
        qpos_cmd = self.sim.get_qpos(q_cmd_ros)
        with self.lock:
            self._pending_qpos_cmd = qpos_cmd

    def _on_cartesian_cmd(self, msg):
        """Receive direct Cartesian fingertip targets — queue for control loop."""
        num_sites = len(self.policy.controller.config.site_names)
        if len(msg.poses) != num_sites:
            rospy.logwarn_throttle(
                1.0,
                "PoseArray has %d poses, expected %d", len(msg.poses), num_sites,
            )
            return
        positions = np.zeros((num_sites, 3), dtype=np.float32)
        orientations = [None] * num_sites
        for i, pose in enumerate(msg.poses):
            positions[i] = [pose.position.x, pose.position.y, pose.position.z]
            q = pose.orientation
            if not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0):
                orientations[i] = (q.x, q.y, q.z, q.w)
        with self.lock:
            self._pending_cartesian_cmd = (positions, orientations)

    def _apply_pending_commands(self):
        """Apply queued commands in the control loop thread (thread-safe)."""
        with self.lock:
            qpos_cmd = self._pending_qpos_cmd
            cartesian_cmd = self._pending_cartesian_cmd
            self._pending_qpos_cmd = None
            self._pending_cartesian_cmd = None

        if qpos_cmd is not None:
            self.policy.controller.sync_qpos(qpos_cmd)
            x_target = self.policy.controller.get_x_obs()
            self.policy.pose_command = x_target.copy()
            self.policy.base_pose_command = x_target.copy()
        elif cartesian_cmd is not None:
            positions, orientations = cartesian_cmd
            x_target = self.policy.controller.get_x_obs().copy()
            x_target[:, :3] = positions
            for i, ori in enumerate(orientations):
                if ori is not None:
                    from scipy.spatial.transform import Rotation as R
                    x_target[i, 3:6] = R.from_quat(ori).as_rotvec().astype(np.float32)
            self.policy.pose_command = x_target.copy()
            self.policy.base_pose_command = x_target.copy()

    def _control_loop(self):
        """Main loop — identical to run_policy.py's run_policy()."""
        next_tick = time.monotonic()
        dt = 1.0 / self.frequency

        while not rospy.is_shutdown():
            t0 = time.monotonic()
            try:
                obs = self.sim.get_observation()

                # Zero the time so policy's prep trajectory works correctly
                if self.start_time is None:
                    self.start_time = float(obs.time)
                obs.time -= self.start_time

                self._apply_gravity()
                self._apply_pending_commands()

                # Step MCC policy (prep interpolation → alignment → compliance)
                action = self.policy.step(obs, self.sim)
                action_arr = np.asarray(action, dtype=np.float32)

                # Write to hardware
                self.sim.set_motor_target(action_arr)
                self.sim.step()
                self.sim.sync()

                # Cache state for ROS publishing
                self.latest_pos = np.asarray(obs.motor_pos, dtype=np.float32)
                self.latest_vel = np.asarray(obs.motor_vel, dtype=np.float32)
                self.latest_cur = np.asarray(
                    obs.motor_cur if obs.motor_cur is not None
                    else np.zeros(16), dtype=np.float32)
                self.latest_wrenches = {
                    site: np.asarray(wrench, dtype=np.float32).copy()
                    for site, wrench in self.policy.wrenches_by_site.items()
                }
                self.latest_site_positions = {
                    site: np.asarray(
                        self.policy.controller.wrench_sim.data.site_xpos[site_id],
                        dtype=np.float32,
                    ).copy()
                    for site, site_id in self.site_ids.items()
                }

                # Aggregate per-fingertip wrenches to net wrench at palm origin
                palm_pos_w = np.asarray(
                    self.policy.controller.wrench_sim.data.xpos[self.palm_body_id],
                    dtype=np.float64,
                ) if self.palm_body_id >= 0 else np.zeros(3)
                net_force = np.zeros(3, dtype=np.float64)
                net_torque = np.zeros(3, dtype=np.float64)
                for site, wrench in self.latest_wrenches.items():
                    f_i = wrench[:3].astype(np.float64)
                    net_force += f_i
                    site_pos = self.latest_site_positions.get(site)
                    if site_pos is not None:
                        r_i = site_pos.astype(np.float64) - palm_pos_w
                        net_torque += np.cross(r_i, f_i)
                    if wrench.shape[0] >= 6:
                        net_torque += wrench[3:6].astype(np.float64)
                self.latest_net_wrench = np.concatenate(
                    [net_force, net_torque]
                ).astype(np.float32)

            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")

            t_proc = time.monotonic()

            # Rate limit (same as run_policy.py)
            next_tick += dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

            t_end = time.monotonic()
            proc_ms = (t_proc - t0) * 1000.0
            actual_dt = t_end - t0
            actual_hz = 1.0 / actual_dt if actual_dt > 0 else 0.0
            max_hz = 1.0 / (t_proc - t0) if (t_proc - t0) > 0 else 0.0
            rospy.loginfo_throttle(
                5.0,
                "control_loop Max rate: %.1f Hz (%.1f ms), Actual rate: %.1f Hz",
                max_hz, proc_ms, actual_hz,
            )

    def _publish_state(self, _event=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.latest_pos.tolist()
        msg.velocity = self.latest_vel.tolist()
        msg.effort = self.latest_cur.tolist()
        self.state_pub.publish(msg)
        self._publish_net_wrench(msg.header.stamp)
        self._publish_gravity_debug(msg.header.stamp)
        if self.publish_debug_markers:
            self._publish_debug_markers(msg.header.stamp)

    def _publish_net_wrench(self, stamp):
        """Publish aggregated net wrench at palm origin in palm-local frame."""
        w = self.latest_net_wrench
        # Transform from world frame to palm-local frame
        palm_rot = np.eye(3, dtype=np.float32)
        if self.palm_body_id >= 0:
            palm_rot = np.asarray(
                self.policy.controller.wrench_sim.data.xmat[self.palm_body_id],
                dtype=np.float32,
            ).reshape(3, 3)
        force_local = self._vec_world_to_local(w[:3], palm_rot)
        torque_local = self._vec_world_to_local(w[3:], palm_rot)
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.debug_frame
        msg.wrench.force.x = float(force_local[0])
        msg.wrench.force.y = float(force_local[1])
        msg.wrench.force.z = float(force_local[2])
        msg.wrench.torque.x = float(torque_local[0])
        msg.wrench.torque.y = float(torque_local[1])
        msg.wrench.torque.z = float(torque_local[2])
        self.net_wrench_pub.publish(msg)

    def _publish_gravity_debug(self, stamp):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.debug_frame
        gravity = self.latest_gravity
        if self.palm_body_id >= 0:
            palm_rot = np.asarray(
                self.policy.controller.wrench_sim.data.xmat[self.palm_body_id],
                dtype=np.float32,
            ).reshape(3, 3)
            gravity = self._vec_world_to_local(gravity, palm_rot)
        msg.vector.x = float(gravity[0])
        msg.vector.y = float(gravity[1])
        msg.vector.z = float(gravity[2])
        self.gravity_pub.publish(msg)

    def _make_arrow_marker(self, marker_id, ns, stamp, start, vec, scale, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.debug_frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.scale.y = 0.008
        marker.scale.z = 0.012
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(0.2)

        p0 = start.reshape(3)
        p1 = (start + vec * scale).reshape(3)
        from geometry_msgs.msg import Point
        marker.points = [
            Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
            Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
        ]
        return marker

    @staticmethod
    def _world_to_local(point_world, origin_world, rot_world_from_local):
        return rot_world_from_local.T @ (point_world - origin_world)

    @staticmethod
    def _vec_world_to_local(vec_world, rot_world_from_local):
        return rot_world_from_local.T @ vec_world

    def _publish_debug_markers(self, stamp):
        markers = MarkerArray()
        palm_pos = np.zeros(3, dtype=np.float32)
        palm_rot = np.eye(3, dtype=np.float32)
        if self.palm_body_id >= 0:
            palm_pos = np.asarray(
                self.policy.controller.wrench_sim.data.xpos[self.palm_body_id],
                dtype=np.float32,
            ).copy()
            palm_rot = np.asarray(
                self.policy.controller.wrench_sim.data.xmat[self.palm_body_id],
                dtype=np.float32,
            ).reshape(3, 3)
        markers.markers.append(
            self._make_arrow_marker(
                0,
                'gravity',
                stamp,
                np.zeros(3, dtype=np.float32),
                self._vec_world_to_local(self.latest_gravity, palm_rot),
                self.debug_gravity_scale,
                (0.2, 0.6, 1.0, 0.9),
            )
        )

        # Net wrench arrow at palm origin (green)
        net_force_local = self._vec_world_to_local(
            self.latest_net_wrench[:3], palm_rot
        )
        markers.markers.append(
            self._make_arrow_marker(
                100,
                'net_wrench',
                stamp,
                np.zeros(3, dtype=np.float32),
                net_force_local,
                self.debug_force_scale,
                (0.2, 1.0, 0.3, 0.9),
            )
        )
        # Net torque arrow at palm origin (cyan)
        net_torque_local = self._vec_world_to_local(
            self.latest_net_wrench[3:], palm_rot
        )
        markers.markers.append(
            self._make_arrow_marker(
                101,
                'net_wrench',
                stamp,
                np.zeros(3, dtype=np.float32),
                net_torque_local,
                self.debug_force_scale,
                (0.0, 0.8, 0.8, 0.9),
            )
        )

        for idx, site in enumerate(sorted(self.latest_wrenches.keys()), start=1):
            wrench = self.latest_wrenches[site]
            site_pos = self.latest_site_positions.get(site)
            if site_pos is None or wrench.shape[0] < 3:
                continue
            markers.markers.append(
                self._make_arrow_marker(
                    idx,
                    'wrench_force',
                    stamp,
                    self._world_to_local(site_pos, palm_pos, palm_rot),
                    self._vec_world_to_local(wrench[:3], palm_rot),
                    self.debug_force_scale,
                    (1.0, 0.3, 0.2, 0.9),
                )
            )

        self.wrench_marker_pub.publish(markers)

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
