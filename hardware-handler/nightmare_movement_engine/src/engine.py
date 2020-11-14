#!/usr/bin/env python3

import time
import numpy as np

import movements
from robot_config.config import DEFAULT_POSE
import rospy
from std_msgs.msg import Byte, Header
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
        self.state = 0  # actual engine state
        self.prev_state = 0  # previous engine state
        self.rate = rospy.Rate(60)  # engine frame rate
        self.angles = np.zeros(shape=(6, 3))  # angle matrix
        self.angles_array = [0]*19  # angle array
        self.states = [0]*19  # all servos disconnected at start
        self.hw_pose = np.zeros(shape=(6, 3))  # initialize as empty and fill it later in set_hw_joint_state callback
        self.pose = DEFAULT_POSE  # init as empty to fill later in callbacks

        self.joint_angle_publisher = rospy.Publisher('engine_angle_joint_states', JointState, queue_size=10)
        self.joint_angle_msg = JOINTSTATE_MSG  # joint angle topic structure
        self.joint_state_publisher = rospy.Publisher('engine_state_joint_states', JointState, queue_size=10)
        self.joint_state_msg = JOINTSTATE_MSG  # joint state topic structure

    def run(self):
        while not rospy.is_shutdown():
            # if(self.state == 0):
            #    movements.sleep(self)
            movements.test(self)
            self.publish_joints()
            self.rate.sleep()

    def publish_joints(self):
        self.joint_angle_msg.position = self.angles_array
        self.joint_angle_msg.header.stamp = rospy.Time.now()
        self.joint_angle_publisher.publish(self.joint_angle_msg)

    def publish_states(self):
        self.joint_state_msg.position = self.states
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(self.joint_state_msg)

    def set_state(self, msg):
        self.state = msg.data

    def set_hw_joint_state(self, msg):
        print(msg)
        # self.hw_pose = msg.data


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)
    rospy.Subscriber("/joint_states", JointState, engine.set_hw_joint_state)

    engine.run()
