#!/usr/bin/env python3

import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from modules import movements
from modules.config import DEFAULT_POSE
from modules.robot_math import abs_ang2pos, abs_pos2ang
from nightmare_state_broadcaster.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import Header

import rospy
from sensor_msgs.msg import JointState


JOINTSTATE_MSG = JointState(header=Header(),
                            name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                  'leg2coxa', 'leg2femur', 'leg2tibia',
                                  'leg3coxa', 'leg3femur', 'leg3tibia',
                                  'leg4coxa', 'leg4femur', 'leg4tibia',
                                  'leg5coxa', 'leg5femur', 'leg5tibia',
                                  'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                            velocity=[],
                            effort=[])


class engineNode():
    def __init__(self):
        self.state = 'sleep'  # actual engine state
        self.prev_state = 'sleep'  # previous engine state
        self.body_displacement = [0] * 6
        self.walk_direction = [0] * 3

        self.angles = np.zeros(shape=(6, 3))  # angle matrix
        self.angles_array = [0] * 19  # angle array
        self.hw_angles_array = [0] * 19
        self.hw_angles = np.zeros(shape=(6, 3))
        self.states = [0] * 19  # all servos disconnected at start
        self.hw_pose = np.zeros(shape=(6, 3))  # initialize as empty and fill it later in set_hw_joint_state callback
        self.pose = DEFAULT_POSE.copy()  # init as empty to fill later in callbacks

        self.joint_angle_publisher = rospy.Publisher('engine_angle_joint_states', JointState, queue_size=10)
        self.joint_angle_msg = JOINTSTATE_MSG  # joint angle topic structure
        self.joint_state_publisher = rospy.Publisher('engine_state_joint_states', JointState, queue_size=10)
        self.joint_state_msg = JOINTSTATE_MSG  # joint state topic structure

    def compute_ik(self):
        self.angles_array = abs_pos2ang(self.pose)
        self.publish_joints()

    def run(self):
        rospy.wait_for_message("/joint_states", JointState)  # at start wait for first hardware frame to know where to start
        rospy.wait_for_message("/nightmare/command", command)

        while not rospy.is_shutdown():
            if self.state == 'stand' and self.prev_state == 'sleep':
                movements.stand_up(self)
            elif self.state == 'sleep' and self.prev_state == 'stand':
                movements.sit(self)
            if self.state == 'stand':
                movements.stand(self)
            elif self.state == 'sleep':
                movements.sleep(self)
                
            self.prev_state = self.state
            rospy.sleep(0.02)

    def publish_joints(self):
        self.joint_angle_msg.position = self.angles_array
        self.joint_angle_msg.header.stamp = rospy.Time.now()
        self.joint_angle_publisher.publish(self.joint_angle_msg)

    def publish_states(self):
        self.joint_state_msg.position = self.states
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(self.joint_state_msg)

    def set_state(self, msg):
        self.state = msg.state
        self.walk_direction = msg.walk_direction
        self.body_displacement = msg.body_displacement

    def set_hw_joint_state(self, msg):
        self.hw_angles_array = msg.position
        self.hw_angles = np.reshape(self.hw_angles_array[:-1], newshape=(6, 3))
        self.hw_pose = abs_ang2pos(self.hw_angles_array)


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)
    rospy.Subscriber("/joint_states", JointState, engine.set_hw_joint_state)

    engine.run()
