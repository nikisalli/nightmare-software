#!/usr/bin/env python3

import time
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

rostopic = ['/nightmare/leg1coxa_position_controller/command',
            '/nightmare/leg1femur_position_controller/command',
            '/nightmare/leg1tibia_position_controller/command',
            '/nightmare/leg2coxa_position_controller/command',
            '/nightmare/leg2femur_position_controller/command',
            '/nightmare/leg2tibia_position_controller/command',
            '/nightmare/leg3coxa_position_controller/command',
            '/nightmare/leg3femur_position_controller/command',
            '/nightmare/leg3tibia_position_controller/command',
            '/nightmare/leg4coxa_position_controller/command',
            '/nightmare/leg4femur_position_controller/command',
            '/nightmare/leg4tibia_position_controller/command',
            '/nightmare/leg5coxa_position_controller/command',
            '/nightmare/leg5femur_position_controller/command',
            '/nightmare/leg5tibia_position_controller/command',
            '/nightmare/leg6coxa_position_controller/command',
            '/nightmare/leg6femur_position_controller/command',
            '/nightmare/leg6tibia_position_controller/command',
            '/nightmare/tail_joint_position_controller/command']

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
        self.angles_array = [0]*19  # angle array
        self.hw_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.joint_angle_msg = JOINTSTATE_MSG
        self.publishers = []
        for topic in rostopic:
            rospy.loginfo(f"subscribing to {topic}")
            self.publishers.append(rospy.Publisher(topic, Float64, queue_size=1))

    def publish_engine_joint_state(self, msg):
        for publisher, angle in zip(self.publishers, msg.position):
            publisher.publish(angle)

    def publish_hw_joint_state(self, msg):
        self.joint_angle_msg.position = np.asarray(msg.position)
        self.joint_angle_msg.header.stamp = rospy.Time.now()
        self.hw_publisher.publish(self.joint_angle_msg)


if __name__ == '__main__':
    rospy.init_node('joint_state_redirector')

    rospy.loginfo("starting engine node")
    engine = Node()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/engine_angle_joint_states", JointState, engine.publish_engine_joint_state)
    rospy.Subscriber("/nightmare/joint_states", JointState, engine.publish_hw_joint_state)
    while not rospy.is_shutdown():
        time.sleep(0.3)
