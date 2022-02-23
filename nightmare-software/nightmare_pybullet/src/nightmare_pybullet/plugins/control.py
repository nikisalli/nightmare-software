#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

URDF_JOINT_OFFSETS = np.array([0.7854, -1.2734, -0.7854, 0, -1.2734, -0.7854, -0.7854, -1.2734, -0.7854, 0.7854, -1.2734, -0.7854, 0, -1.2734, -0.7854, -0.7854, -1.2734, -0.7854])


class Control:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # joint names
        self.joint_names = {**kargs['rev_joints'], **kargs['prism_joints']}
        self.joint_indices = [n for n in self.joint_names]
        self.joint_number = len(self.joint_indices)
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = [0] * self.joint_number
        self.velocity_joint_commands = [0] * self.joint_number
        self.effort_joint_commands = [0] * self.joint_number
        # subscriber
        self.joint_state_subscriber = rospy.Subscriber('/engine/joint_states', JointState, self.joint_state_cb, queue_size=1)
        # joint max efforts
        if rospy.has_param('~max_effort_vel_mode'):
            rospy.logwarn('max_effort_vel_mode parameter is deprecated, please use max_effort instead')
            # kept for backwards compatibility, delete after some time
            self.max_effort = rospy.get_param('~max_effort_vel_mode', 10000.0)
        else:
            self.max_effort = rospy.get_param('~max_effort', 10000.0)

        if rospy.has_param('~min_effort_vel_mode'):
            rospy.logwarn('min_effort_vel_mode parameter is deprecated, please use min_effort instead')
            # kept for backwards compatibility, delete after some time
            self.min_effort = rospy.get_param('~min_effort_vel_mode', 0.0)
        else:
            self.min_effort = rospy.get_param('~min_effort', 0.0)

        self.force_commands = [self.max_effort] * self.joint_number

    def joint_state_cb(self, msg):
        # add offset REAL -> URDF
        self.position_joint_commands = np.array(msg.position) + URDF_JOINT_OFFSETS
        self.force_commands = np.array(msg.effort) * self.max_effort + (np.ones(shape=len(msg.effort)) - np.array(msg.effort)) * self.min_effort
        # self.velocity_joint_commands = msg.velocity
        # self.effort_joint_commands = msg.effort

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        # forward commands to pybullet, give priority to position control cmds, then vel, at last effort
        print(self.position_joint_commands, self.joint_indices, self.joint_names)
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                          controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.force_commands)
        # self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
        #                                   controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.force_commands)
        # self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
        #                                   controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
