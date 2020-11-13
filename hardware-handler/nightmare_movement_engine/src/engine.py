#!/usr/bin/env python3

import time
import numpy as np

import rospy
from std_msgs.msg import Byte, Header
from sensor_msgs.msg import JointState
from movements import stand_up


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

    def run(self):
        while not rospy.is_shutdown():
            if(self.state == 0):
                pass
                # movements.sleep()

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


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)

    engine.run()
