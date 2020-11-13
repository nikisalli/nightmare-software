#!/usr/bin/env python3

import time
import numpy as np

import movements

import rospy
from std_msgs.msg import Byte, Header
from sensor_msgs.msg import JointState


class engineNode():
    def __init__(self):
        self.state = 0
        self.prev_state = 0
        self.rate = rospy.Rate(60)  # engine frame rate
        self.joint_angle_publisher = rospy.Publisher('engine_angle_joint_states', JointState, queue_size=10)
        self.joint_angle_msg = JointState(header=Header(),
                                          name=['leg3coxa', 'leg2coxa', 'leg1coxa', 'leg6coxa', 'leg5coxa', 'leg4coxa',
                                                'leg3femur', 'leg2femur', 'leg1femur', 'leg6femur', 'leg5femur', 'leg4femur',
                                                'leg3tibia', 'leg2tibia', 'leg1tibia', 'leg6tibia', 'leg5tibia', 'leg4tibia',
                                                'tail_joint'],
                                          velocity=[],
                                          effort=[])  # joint angle topic structure

        self.joint_state_publisher = rospy.Publisher('engine_state_joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState(header=Header(),
                                          name=['leg3coxa', 'leg2coxa', 'leg1coxa', 'leg6coxa', 'leg5coxa', 'leg4coxa',
                                                'leg3femur', 'leg2femur', 'leg1femur', 'leg6femur', 'leg5femur', 'leg4femur',
                                                'leg3tibia', 'leg2tibia', 'leg1tibia', 'leg6tibia', 'leg5tibia', 'leg4tibia',
                                                'tail_joint'],
                                          velocity=[],
                                          effort=[])  # joint state topic structure
        self.hw_pose = [0]*19  # initialize as empty and fill it later in set_hw_joint_state callback
        self.pose = [0]*19  # init as empty to fill later in callbacks
        self.states = [0]*19  # all servos disconnected while starting

    def run(self):
        while not rospy.is_shutdown():
            if(self.state == 0):
                movements.sleep(self)

            self.publish_joints()
            self.rate.sleep()

    def publish_joints(self):
        angles = [0]*19
        self.joint_angle_msg.position = angles
        self.joint_angle_msg.header.stamp = rospy.Time.now()
        self.joint_angle_publisher.publish(self.joint_angle_msg)

    def publish_states(self):
        states = [0]*19
        self.joint_state_msg.position = states
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(self.joint_state_msg)

    def set_state(self, msg):
        self.state = msg.data

    def set_hw_joint_state(self, msg):
        self.pose = msg.data


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)
    rospy.Subscriber("/joint_states", JointState, engine.set_hw_joint_state)

    engine.run()
