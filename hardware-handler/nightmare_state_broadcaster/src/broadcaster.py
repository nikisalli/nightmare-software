#!/usr/bin/env python3

# standard python imports
import time

# ros imports
import rospy
from std_msgs.msg import Byte
from nightmare_usb_joystick.msg import joystick  # noqa
from threading import Thread

state = 0
actual_topic = ''


def callback_joystick(msg):
    pass
    # rospy.loginfo(str(msg))


def find_talker():
    unparsed_topic_list = rospy.get_published_topics(namespace='/control')
    if len(unparsed_topic_list) == 0:
        rospy.loginfo("nightmare_state_publisher couldn't find any listenable topics!")
        return [None, None]
    parsed_topic_list = [topic[0] for topic in unparsed_topic_list]

    # in future more options will be here
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

    def run(self):
        while not rospy.is_shutdown():
            while None in self.topic and not rospy.is_shutdown():  # wait for topic or for process to die
                self.topic = find_talker()
                time.sleep(0.5)

            rospy.loginfo("subscribing...")
            self.old_topic = self.topic
            sub = rospy.Subscriber(self.old_topic[0], self.old_topic[1], callback_joystick)

            while self.old_topic == self.topic:
                self.topic = find_talker()
                time.sleep(0.05)

            rospy.loginfo("searching for new topic!")
            sub.unregister()


def handle_state():
    rate = rospy.Rate(50)
    pub_state = rospy.Publisher("/nightmare/state", Byte, queue_size=1)

    while not rospy.is_shutdown():
        pub_state.publish(state)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_broadcaster')

    rospy.loginfo("starting state broadcaster")

    t = ListenerThread()
    t.start()

    handle_state()
