#!/usr/bin/env python3
# pylint: disable=broad-except, no-name-in-module

# standard python imports
import time
from threading import Thread
import sys
import os
import numpy as np

# ros imports
import rospy
from nightmare_state_broadcaster.msg import command
from std_msgs.msg import Header

sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from nightmare_config.config import (MAX_BODY_TRASL,
                                     MAX_BODY_ROT,
                                     MAX_WALK_TRASL_VEL,
                                     MAX_WALK_ROT_VEL,
                                     MAX_BODY_TRASL_CMD_VEL,
                                     MAX_BODY_ROT_CMD_VEL,
                                     MAX_BODY_TRASL_CMD_ACC,
                                     MAX_BODY_ROT_CMD_ACC,
                                     MAX_WALK_TRASL_CMD_ACC,
                                     MAX_WALK_ROT_CMD_ACC)


state = 'sleep'  # string containing the robot's global state e.g. walking sitting etc
gait = 'tripod'  # string containing the robot's walking gait

body_trasl = np.array([0, 0, 0])
body_rot = np.array([0, 0, 0])
walk_trasl = np.array([0, 0, 0])
walk_rot = np.array([0, 0, 0])


def find_talker():
    unparsed_topic_list = rospy.get_published_topics(namespace='/control')
    if len(unparsed_topic_list) == 0:
        rospy.loginfo("nightmare_state_broadcaster couldn't find any listenable topics!")
        return [None, None]
    parsed_topic_list = [topic[0] for topic in unparsed_topic_list]

    # in future more options will be here (one for each control method)
    if '/control/usb_joystick' in parsed_topic_list:
        return ['/control/usb_joystick', command]  # return topic name and topic type
    elif '/control/keyboard' in parsed_topic_list:
        return ['/control/keyboard', command]  # return topic name and topic type
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

            while self.old_topic == self.topic and not rospy.is_shutdown():
                self.topic = find_talker()
                time.sleep(0.05)

            rospy.loginfo("searching for new topic!")
            sub.unregister()

    def callback_joystick(self, msg):  # joystick control logic
        global state
        global walk_trasl
        global walk_rot
        global body_trasl
        global body_rot
        global gait

        body_trasl = np.array(msg.body_trasl) * MAX_BODY_TRASL
        body_rot = np.array(msg.body_rot) * MAX_BODY_ROT

        walk_trasl = np.array(msg.walk_trasl) * MAX_WALK_TRASL_VEL
        walk_rot = np.array(msg.walk_rot) * MAX_WALK_ROT_VEL

        state = msg.state
        gait = msg.gait


def handle_state():
    rate = rospy.Rate(60)
    pub_state = rospy.Publisher("/nightmare/command", command, queue_size=1)
    header = Header()

    final_body_trasl = np.array([0., 0., 0.])
    final_body_rot = np.array([0., 0., 0.])
    final_walk_trasl = np.array([0., 0., 0.])
    final_walk_rot = np.array([0., 0., 0.])

    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()

        # TODO remove this bs because c'mon you can do better
        # ps it's a filter for commands
        final_body_trasl += (body_trasl - final_body_trasl) * 0.1
        final_body_rot += (body_rot - final_body_rot) * 0.1
        final_walk_trasl += (walk_trasl - final_walk_trasl) * 0.1
        final_walk_rot += (walk_rot - final_walk_rot) * 0.1

        pub_state.publish(command(header, final_body_trasl, final_body_rot, final_walk_trasl, final_walk_rot, state, gait))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_broadcaster')

    rospy.loginfo("starting state broadcaster")

    t = ListenerThread()
    t.start()

    handle_state()
