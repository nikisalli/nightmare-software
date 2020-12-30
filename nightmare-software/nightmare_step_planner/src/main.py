#!/usr/bin/env python3

# standard python imports
import sys
import os

# ros imports
import rospy
from nightmare_step_planner.msg import footsteps  # pylint: disable=no-name-in-module
from nightmare_step_planner.msg import command  # pylint: disable=no-name-in-module

from std_msgs.msg import Header

sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from config import *


EPSILON = 0.001  # for float comparisons


def feq(a, b):  # floating point equal
    return abs(a - b) < EPSILON


class stepPlannerNode():
    def __init__(self):
        self.state = 'sleep'  # actual engine state
        self.prev_state = 'sleep'  # previous engine state
        self.body_displacement = [0] * 6
        self.walk_direction = [0] * 3
        self.rate = rospy.Rate(50)

    def run(self):
        pub_state = rospy.Publisher("/nightmare/footsteps", footsteps, queue_size=1)
        header = Header()

        while not rospy.is_shutdown():
            header.stamp = rospy.Time.now()
            pub_state.publish(footsteps(header, 'lol'))
            self.rate.sleep()

    def set_state(self, msg):
        self.state = msg.state
        self.walk_direction = msg.walk_direction
        self.body_displacement = msg.body_displacement


if __name__ == '__main__':
    rospy.init_node('step_planner')

    rospy.loginfo("starting state broadcaster")
    engine = stepPlannerNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)

    engine.run()
