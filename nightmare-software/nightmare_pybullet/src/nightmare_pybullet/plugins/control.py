#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


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
            max_effort = rospy.get_param('~max_effort_vel_mode', 100.0)
        else:
            max_effort = rospy.get_param('~max_effort', 100.0)
        self.force_commands = [max_effort] * self.joint_number

    def joint_state_cb(self, msg):
        self.position_joint_commands = msg.position
        # self.velocity_joint_commands = msg.velocity
        # self.effort_joint_commands = msg.effort

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        # forward commands to pybullet, give priority to position control cmds, then vel, at last effort
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                          controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.force_commands)
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                          controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.force_commands)
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                          controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
