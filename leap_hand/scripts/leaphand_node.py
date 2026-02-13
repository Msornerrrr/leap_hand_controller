#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg
import os
import sys
import time
from typing import Optional

import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *


def get_ros_loop_rate_str(
    start_time: rospy.Time,
    before_sleep_time: rospy.Time,
    after_sleep_time: rospy.Time,
    node_name: Optional[str] = None,
) -> str:
    max_rate_dt = (before_sleep_time - start_time).to_sec()
    max_rate_hz = 1 / max_rate_dt
    actual_rate_dt = (after_sleep_time - start_time).to_sec()
    actual_rate_hz = 1 / actual_rate_dt
    loop_rate_str = f"Max rate: {np.round(max_rate_hz, 1)} Hz ({np.round(max_rate_dt * 1000, 1)} ms), Actual rate: {np.round(actual_rate_hz, 1)} Hz"
    return f"{node_name} {loop_rate_str}" if node_name is not None else loop_rate_str


#LEAP hand conventions:

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

# Author recommends only to query when necessary and below 90 samples a second.  
# Use the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.


class LeapNode:
    def __init__(self, frequency ):
        ####Some parameters to control the hand #! Reduce PD values for less jittery control, Increase for more strength
        self.kP = float(rospy.get_param('/leaphand_node/kP', 800.0)) 
        self.kI = float(rospy.get_param('/leaphand_node/kI', 0.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 200.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 550.0)) #don't go past 600ma on this, or it'll overcurrent sometimes for regular, 350ma for lite.
        self.max_delta_q = float(rospy.get_param('/leaphand_node/max_delta_q', 0.15)) # Max joint position change per step (radians). Set to 0 to disable.
        self.hand = rospy.get_param('/leaphand_node/hand', 'right')
        self.prev_pos = self.pos = self.curr_pos = np.zeros(16)
        self.frequency = frequency
        self.lock = threading.Lock()

        # Internal state variables
        self.cmd_pose = np.zeros(16)  # latest commanded pose (pre-offset), updated by subscriber
        self.latest_pos = np.zeros(16)
        self.latest_vel = np.zeros(16)
        self.latest_eff = np.zeros(16)

        #subscribes to a variety of sources that can command the hand, and creates services that can give information about the hand out
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose, queue_size=1)

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        self.joint_names = [f"joint_{i}" for i in self.motors]

        ports = {'right': "/dev/ttyLEAP_RIGHT", 'left': "/dev/ttyLEAP_LEFT"}
        port = ports.get(self.hand)
        if port is None:
            rospy.logfatal(f"Invalid hand parameter: '{self.hand}'. Must be 'left' or 'right'.")
            sys.exit(1)

        try:
            self.dxl_client = DynamixelClient(motors, port, 4000000)
            self.dxl_client.connect()
            rospy.loginfo(f"Connected to {self.hand} hand on {port}")
        except Exception as e:
            rospy.logfatal(f"Failed to connect to {port}: {e}")
            sys.exit(1)
        
        # Enables position-current control mode, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.sync_write(motors, np.zeros(len(motors)), 9, 1) # Set return time delay to 0
        self.dxl_client.sync_write(motors, np.ones(len(motors))*125, 112, 4) # Velocity 
        self.dxl_client.sync_write(motors, np.ones(len(motors))*0, 108, 4) # Acceleration
        self.dxl_client.set_torque_enabled(motors, True)

        # Set parameters for PID control
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less

        #Max at current (in unit 1mA) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)

        # Get Min and max
        self.min, self.max = lhu.LEAP_limits(self.hand)
        try: 
            #Move motors to 0 position
            self.set_initial_position(self.curr_pos)
            time.sleep(1.0) # Wait before reading positions

            # Read initial state from hardware after delay
            output = self.dxl_client.read_pos_vel()
            self.latest_pos = output[0] - np.pi
            self.latest_vel = output[1]

            # Setup timer for periodic publishing (60 Hz default)
            self.publish_rate = float(rospy.get_param('/leaphand_node/publish_rate', self.frequency))
            self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_state)

            # Initialize services after everything is ready
            rospy.Service('leap_position', leap_position, self.pos_srv)
            rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
            # rospy.Service('leap_effort', leap_effort, self.eff_srv)
            rospy.Service('leap_pos_vel', leap_pos_vel, self.pos_vel_srv)
            # rospy.Service('leap_pos_vel_eff', leap_pos_vel_eff, self.pos_vel_eff_srv)

            # Publish state of hand every time you fullfill a service
            self.state_pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=10)

            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            while not rospy.is_shutdown():
                rospy.spin()
        finally:
            self.dxl_client.set_torque_enabled(motors, False)

    def control_loop(self):
        """Runs at 60Hz: read state, clamp command, write to hardware."""
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            try:
                # 1. Read current joint state from hardware
                pos, vel = self.dxl_client.read_pos_vel()
                self.latest_pos = pos - np.pi
                self.latest_vel = vel

                # 2. Grab the latest commanded pose
                with self.lock:
                    pose = self.cmd_pose.copy()

                # 3. Apply delta clamping relative to actual joint state
                if self.max_delta_q > 0:
                    delta = np.clip(pose - self.latest_pos, -self.max_delta_q, self.max_delta_q)
                    pose = self.latest_pos + delta

                # 4. Write to hardware (with pi offset for LEAP convention)
                self.prev_pos = self.curr_pos
                self.curr_pos = pose + np.pi
                self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
            except Exception as e:
                rospy.logerr(f"Hardware communication error: {e}")
            before_sleep_time = rospy.Time.now()
            rate.sleep()
            after_sleep_time = rospy.Time.now()
            rospy.loginfo_throttle(5.0, get_ros_loop_rate_str(start_time, before_sleep_time, after_sleep_time, "control_loop"))
            

    def _receive_pose(self, pose):
        """Receive commanded pose and store it. The control_loop handles writing."""
        pose = np.array(pose.position)
        pose = np.clip(pose, self.min, self.max)
        with self.lock:
            self.cmd_pose = pose


    def set_initial_position(self, pose):
        """Set initial position. Direct write since control_loop hasn't started yet."""
        self.cmd_pose = np.clip(pose, self.min, self.max)
        self.curr_pos = self.cmd_pose + np.pi
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)


    #Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, req):
        return {"position": self.latest_pos}

    #Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, req):
        return {"velocity": self.latest_vel}

    # #Service that reads and returns the effort/current of the robot in LEAP Embodiment
    # def eff_srv(self, req):
    #     eff = self.dxl_client.read_cur()
    #     self.latest_eff = eff
    #     return {"effort": self.latest_eff}

    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_srv(self, req):
        return {"position": self.latest_pos, "velocity": self.latest_vel}

    # #Use these combined services to save a lot of latency if you need multiple datapoints
    # def pos_vel_eff_srv(self, req):
    #     pos, vel, eff = self.dxl_client.read_pos_vel_cur()
    #     self.latest_pos = pos - np.pi
    #     self.latest_vel = vel
    #     self.latest_eff = eff
    #     return {"position": self.latest_pos, "velocity": self.latest_vel, "effort": self.latest_eff}

    def publish_state(self, event=None):
        """Publish the current internal state without reading from hardware."""
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.name = self.joint_names
        state.position = self.latest_pos
        state.velocity = self.latest_vel
        state.effort = self.latest_eff
        self.state_pub.publish(state)

import argparse
def make_args():
    parser = argparse.ArgumentParser(
        description="Process the args"
    )

    parser.add_argument(
        "--frequency",
        type=float,
        default=80.0, 
        help="Frequency for Reading the Motor position and velocity",
    )

    args = parser.parse_args()
    return args

if __name__ == "__main__":
    # Filter Ros Args
    ros_args = [arg for arg in sys.argv if arg.startswith('__')]
    clean_argv = [arg for arg in sys.argv if not arg.startswith('__')]

    # Temporarily replace sys.argv to exclude ROS arguments
    sys.argv = clean_argv
    args = make_args()

    rospy.init_node("leaphand_node")
    leaphand_node = LeapNode(args.frequency)
