#!/usr/bin/env python3

import time

import rospy
from std_msgs.msg import Byte, Header
from sensor_msgs.msg import JointState
from movements import stand_up


class engineNode():
    def __init__(self):
        self.state = 0
        self.prev_state = 0
        self.joint_publisher = rospy.Publisher('engine_joint_states', JointState, queue_size=10)
        self.joint_msg = JointState(header=Header(),
                                    name=['Rev103', 'Rev104', 'Rev105', 'Rev106', 'Rev107', 'Rev108',
                                          'Rev109', 'Rev110', 'Rev111', 'Rev112', 'Rev113', 'Rev114',
                                          'Rev115', 'Rev116', 'Rev117', 'Rev118', 'Rev119', 'Rev120', 'Rev121'],
                                    velocity=[],
                                    effort=[])  # joint topic structure

    def run(self):
        while not rospy.is_shutdown():
            if(self.state == 0):
                time.sleep(1000)
                # movements.sleep()

    def publish_joints(self):
        angles = [0]*19
        self.joint_msg.position = angles
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_publisher.publish(self.joint_msg)

    def set_state(self, msg):
        self.state = msg.data


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)

    engine.run()
