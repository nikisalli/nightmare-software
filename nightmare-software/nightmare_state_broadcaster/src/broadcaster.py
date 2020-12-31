#!/usr/bin/env python3

# standard python imports
import time
from threading import Thread
import sys
import os

# ros imports
import rospy
from nightmare_state_broadcaster.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import Header

sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from config import *


EPSILON = 0.001  # for float comparisons


state = 'sleep'  # string containing the robot's global state e.g. walking sitting etc
gait = 'tripod'  # string containing the robot's walking gait

# roll pitch yaw x y z body displacement
body_displacement = [0] * 6

# x y in cm/sec being the resultants of a vector describing direction and modulo (speed) the robot should follow
# yaw in deg/sec being the rotation speed of the robot's static position
walk_direction = [0] * 3


def feq(a, b):  # floating point equal
    return abs(a - b) < EPSILON


def find_talker():
    unparsed_topic_list = rospy.get_published_topics(namespace='/control')
    if len(unparsed_topic_list) == 0:
        rospy.loginfo("nightmare_state_broadcaster couldn't find any listenable topics!")
        return [None, None]
    parsed_topic_list = [topic[0] for topic in unparsed_topic_list]

    # in future more options will be here (one for each control method)
    if '/control/usb_joystick' in parsed_topic_list:
        return ['/control/usb_joystick', command]  # return topic name and topic type
    else:
        rospy.loginfo("control method not yet implemented!")
        return [None, None]


class ListenerThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.topic = [None, None]
        self.old_topic = [None, None]

    def run(self):
        while not rospy.is_shutdown():
            while None in self.topic and not rospy.is_shutdown():  # wait for topic or for process to die
                self.topic = find_talker()
                time.sleep(0.5)

            rospy.loginfo("subscribing...")
            self.old_topic = self.topic
            sub = rospy.Subscriber(self.old_topic[0], self.old_topic[1], self.callback_joystick)

            while self.old_topic == self.topic:
                self.topic = find_talker()
                time.sleep(0.05)

            rospy.loginfo("searching for new topic!")
            sub.unregister()

    def callback_joystick(self, msg):  # joystick control logic
        global state
        global walk_direction
        global body_displacement
        global gait

        walk_direction = msg.walk_direction
        body_displacement = msg.body_displacement
        state = msg.state
        gait = msg.gait


def handle_state():
    rate = rospy.Rate(50)
    pub_state = rospy.Publisher("/nightmare/command", command, queue_size=1)
    header = Header()

    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        pub_state.publish(command(header, body_displacement, walk_direction, state, gait))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_broadcaster')

    rospy.loginfo("starting state broadcaster")

    t = ListenerThread()
    t.start()

    handle_state()
