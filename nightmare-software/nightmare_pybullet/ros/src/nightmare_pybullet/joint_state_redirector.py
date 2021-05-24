#!/usr/bin/env python3

import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

rostopic = ['leg1coxa_position_controller/command',
            'leg1femur_position_controller/command',
            'leg1tibia_position_controller/command',
            'leg2coxa_position_controller/command',
            'leg2femur_position_controller/command',
            'leg2tibia_position_controller/command',
            'leg3coxa_position_controller/command',
            'leg3femur_position_controller/command',
            'leg3tibia_position_controller/command',
            'leg4coxa_position_controller/command',
            'leg4femur_position_controller/command',
            'leg4tibia_position_controller/command',
            'leg5coxa_position_controller/command',
            'leg5femur_position_controller/command',
            'leg5tibia_position_controller/command',
            'leg6coxa_position_controller/command',
            'leg6femur_position_controller/command',
            'leg6tibia_position_controller/command',
            'tail_joint_position_controller/command']

JOINTSTATE_MSG = JointState(header=Header(),
                            name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                  'leg2coxa', 'leg2femur', 'leg2tibia',
                                  'leg3coxa', 'leg3femur', 'leg3tibia',
                                  'leg4coxa', 'leg4femur', 'leg4tibia',
                                  'leg5coxa', 'leg5femur', 'leg5tibia',
                                  'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                            velocity=[],
                            effort=[])


class Node():
    def __init__(self):
        self.hw_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.joint_angle_msg = JOINTSTATE_MSG
        self.publishers = []
        self.engine_msg = [0] * 19
        self.rate = rospy.Rate(50)
        for topic in rostopic:
            rospy.loginfo(f"generating publisher on topic {topic}")
            self.publishers.append(rospy.Publisher(topic, Float64, queue_size=1))

    def run(self):
        while not rospy.is_shutdown():
            for publisher, angle in zip(self.publishers, self.engine_msg):
                publisher.publish(angle)  # publish to gazebo

            self.joint_angle_msg.position = np.asarray(self.engine_msg)
            self.joint_angle_msg.header.stamp = rospy.Time.now()
            self.hw_publisher.publish(self.joint_angle_msg)  # publish to whatever needs tf

            self.rate.sleep()

    def publish_engine_joint_state(self, msg):
        self.engine_msg = msg.position


def main():
    rospy.init_node('joint_state_redirector')

    rospy.loginfo("starting engine node")
    engine = Node()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/engine/angle_joint_states", JointState, engine.publish_engine_joint_state)

    engine.run()
