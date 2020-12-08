#!/usr/bin/env python3

# standard python imports
import time
from threading import Thread
import sys
import os

# ros imports
import rospy
from nightmare_state_broadcaster.msg import joystick  # pylint: disable=no-name-in-module
from nightmare_state_broadcaster.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import Header

sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from config import *


EPSILON = 0.001  # for float comparisons


state = 'sleep'  # string containing the robot's global state e.g. walking sitting etc

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
        rospy.loginfo("nightmare_state_publisher couldn't find any listenable topics!")
        return [None, None]
    parsed_topic_list = [topic[0] for topic in unparsed_topic_list]

    # in future more options will be here (one for each control method)
    if '/control/usb_joystick' in parsed_topic_list:
        return ['/control/usb_joystick', joystick]  # return topic name and topic type
    else:
        rospy.loginfo("control method not yet implemented!")
        return [None, None]


class ListenerThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.topic = [None, None]
        self.old_topic = [None, None]

        # joystick vars
        self.mode = 'walk'  # string containing joystick control mode
        self.height_displacement = 0
        self.height_change_timer = 0
        self.prev_start = False
        self.prev_ba = False
        self.prev_bb = False
        self.prev_bx = False
        self.prev_by = False

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

        if msg.bb and self.prev_bb is False:  # joystick mode slection
            self.mode = 'walk'
            rospy.loginfo('mode set to walk')
        elif msg.ba and self.prev_ba is False:
            self.mode = 'stand'
            rospy.loginfo('mode set to stand')
        elif msg.bx and self.prev_bx is False:
            self.mode = 'leg control'
            rospy.loginfo('mode set to leg control')
        self.prev_ba = msg.ba
        self.prev_bb = msg.bb
        self.prev_bx = msg.bx
        self.prev_by = msg.by

        if msg.start and state == 'sleep' and self.prev_start is False:
            state = 'stand'
            rospy.loginfo('state set to stand')
        elif msg.start and state == 'stand' and self.prev_start is False:
            state = 'sleep'
            rospy.loginfo('state set to sleep')
        self.prev_start = msg.start

        if (feq(msg.ty, -1) or feq(msg.ty, 1)) and (time.time() - self.height_change_timer) > 0.5:
            self.height_displacement += 0.5 * -msg.ty  # increment by 0.5cm every 0.5s
            if abs(self.height_displacement) > MAX_HEIGHT_DISPLACEMENT:  # if limit exceeded set to limit
                self.height_displacement = MAX_HEIGHT_DISPLACEMENT * msg.ty
            rospy.loginfo(f"height set to {self.height_displacement}")

        if self.mode == 'walk':
            walk_direction = [msg.jlx * MAX_WALK_SPEED_X,
                              -msg.jly * MAX_WALK_SPEED_Y,
                              msg.jrx * MAX_WALK_ROTATIONAL_SPEED]  # for some reason my joystick gives inverted y
            body_displacement = [0, 0, self.height_displacement, 0, 0, 0]
        elif self.mode == 'stand':
            walk_direction = [0, 0, 0]
            body_displacement = [msg.jlx * MAX_X_DISPLACEMENT,
                                 msg.jly * MAX_Y_DISPLACEMENT,
                                 self.height_displacement,
                                 msg.jrx * MAX_ROLL_DISPLACEMENT,
                                 msg.jry * MAX_PITCH_DISPLACEMENT,
                                 0]


def handle_state():
    rate = rospy.Rate(50)
    pub_state = rospy.Publisher("/nightmare/command", command, queue_size=1)
    header = Header()

    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        pub_state.publish(command(header, body_displacement, walk_direction, state))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_broadcaster')

    rospy.loginfo("starting state broadcaster")

    t = ListenerThread()
    t.start()

    handle_state()
