#!/usr/bin/env python3

import time
from threading import Thread
from typing import List, Any

import lewansoul_lx16a
import rospy
import serial
import tf
import tf.transformations
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# robot servo serial ports
tty_list = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]


# simple range mapping function
def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# servo class to hold servo parameters
class Servo:
    def __init__(self):
        self.tty = ""
        self.id = 0
        self.angle = 0
        self.pos = 0


# leg class to hold servo positions
class Leg:
    def __init__(self, num):
        self.num = num
        self.servo = [Servo(), Servo(), Servo()]


class ListenerThread(Thread):
    def __init__(self, tty):
        Thread.__init__(self)
        self.tty = tty

    def run(self):
        while not rospy.is_shutdown():
            for leg_num in range(6):
                for servo_num in range(3):
                    id = leg_list[leg_num].servo[servo_num].id
                    port = leg_list[leg_num].servo[servo_num].tty
                    if port == self.tty:
                        try:
                            pos = controller[port].get_position(id, timeout=0.02)
                            leg_list[leg_num].servo[servo_num].pos = pos
                            leg_list[leg_num].servo[servo_num].angle = round(fmap(pos, 0, 1000, -2.0944, 2.0944), 4)
                        except:
                            rospy.logerr("couldn't read servo position. ID: " + str(id) + " port: ttyUSB" + str(port))
            time.sleep(0.05)  # execute every 0.05s


class HardwareHandlerNode:
    """main node class"""

    def __init__(self, legs: List[Leg], fixed_frame: Any, frame: Any, publisher: rospy.Publisher,
                 broadcaster: tf.TransformBroadcaster, joint_msg: JointState):
        self.legs = legs
        self.fixed_frame = fixed_frame
        self.frame = frame
        self.publisher = publisher
        self.broadcaster = broadcaster
        self.joint_msg = joint_msg

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Ready for publishing")

    def publish(self):
        self.broadcaster.sendTransform((0, 0, 0),
                                       tf.transformations.quaternion_from_euler(0, 0, 0),
                                       rospy.Time.now(),
                                       self.frame,
                                       self.fixed_frame)
        self._publish_jnt()

    def _publish_jnt(self):
        angles = [servo.angle for leg in self.legs for servo in leg.servo]

        self.joint_msg.position = [angles[6], angles[3], angles[0], angles[15], angles[12], angles[9],
                                   -angles[7], -angles[4], -angles[1], -angles[16], -angles[13], -angles[10],
                                   -angles[8], -angles[5], -angles[2], -angles[17], -angles[14], -angles[11], 0]

        self.joint_msg.header.stamp = rospy.Time.now()
        self.publisher.publish(self.joint_msg)

    def on_shutdown(self):
        rospy.loginfo(f"shutting down {type(self).__name__}")


if __name__ == '__main__':
    import os
    import sys

    rospy.loginfo("starting hardware handler node")

    rospy.init_node('hardware_handler')

    # -------------------------------------------

    rospy.loginfo("setting ports to low latency mode")

    for tty in tty_list:
        os.system('setserial ' + tty + ' low_latency')

    # -------------------------------------------

    rospy.loginfo("creating controller objects")

    controller = [0] * 4
    for i, tty in enumerate(tty_list):
        controller[i] = lewansoul_lx16a.ServoController(serial.Serial(tty, 115200, timeout=1))

    # -------------------------------------------

    rospy.loginfo("indexing servos")

    leg_list = []
    for i in range(6):
        leg_list.append(Leg(i))

    for leg_num in range(6):
        for servo_num in range(3):
            found = False
            id = (leg_num * 3) + servo_num + 1
            for tty_num, tty in enumerate(tty_list):
                try:
                    controller[tty_num].get_servo_id(id, timeout=0.05)
                    rospy.loginfo("found servo id " + str(id) + " on " + str(tty))
                    leg_list[leg_num].servo[servo_num].id = id
                    leg_list[leg_num].servo[servo_num].tty = tty_num
                    found = True
                    break
                except:
                    pass
            if(not found):
                rospy.logerr("couldn't find a servo with ID: " + str(id) + " on any serial bus!")
                sys.exit("couldn't find a servo with ID: " + str(id) + " on any serial bus!")

    # -------------------------------------------

    rospy.loginfo("spawning reading threads")

    thread_list = []

    for i in range(len(tty_list)):
        thread_list.append(ListenerThread(i))
        thread_list[i].start()

    node = HardwareHandlerNode(
        legs=leg_list,
        fixed_frame=rospy.get_param('~fixed_frame', 'world'), # set fixed frame relative to world to apply the transform
        frame=rospy.get_param('~frame', "base_link"),  # set frame name
        publisher=rospy.Publisher('joint_states', JointState, queue_size=10),  # joint state publisher
        broadcaster=tf.TransformBroadcaster(),
        joint_msg=JointState(header=Header(),
                             name=['Rev103', 'Rev104', 'Rev105', 'Rev106', 'Rev107', 'Rev108', 'Rev109',
                                 'Rev110', 'Rev111', 'Rev112', 'Rev113', 'Rev114', 'Rev115', 'Rev116',
                                 'Rev117', 'Rev118', 'Rev119', 'Rev120', 'Rev121'],
                             velocity=[],
                             effort=[])  # joint topic structure
    )

    try:
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            node.publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
