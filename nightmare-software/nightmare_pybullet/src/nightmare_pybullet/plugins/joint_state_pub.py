#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /joint_states
"""

import rospy
from sensor_msgs.msg import JointState
import numpy as np


URDF_JOINT_OFFSETS = np.array([0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854])


class joinStatePub:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # get joints names and store them in dictionary
        self.joint_index_name_dic = kargs['rev_joints']
        # register this node in the network as a publisher in /joint_states topic
        self.pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.pub_joint_states_hw = rospy.Publisher('/hardware/joint_states', JointState, queue_size=1)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # setup msg placeholder
        joint_msg = JointState()
        joint_msg_hw = JointState()
        # get joint states
        for joint_index in self.joint_index_name_dic:
            # get joint state from pybullet
            joint_state = self.pb.getJointState(self.robot, joint_index)
            # fill msg
            joint_msg.name.append(self.joint_index_name_dic[joint_index])
            joint_msg.position.append(joint_state[0])
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3])  # applied effort in last sim step

            joint_msg_hw.name.append(self.joint_index_name_dic[joint_index])
            # subtract offset URDF -> REAL
            joint_msg_hw.position.append(joint_state[0] - URDF_JOINT_OFFSETS)
            joint_msg_hw.velocity.append(joint_state[1])
            joint_msg_hw.effort.append(joint_state[3])  # applied effort in last sim step
        # update msg time using ROS time api
        joint_msg.header.stamp = rospy.Time.now()
        # publish joint states to ROS
        self.pub_joint_states.publish(joint_msg)
        self.pub_joint_states_hw.publish(joint_msg_hw)
