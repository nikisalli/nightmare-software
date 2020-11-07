#!/usr/bin/env python

import os
import time
from dataclasses import dataclass
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
TTY_LIST = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]
NUMBER_OF_LEGS = 6
NUMBER_OF_SERVOS = 19
NUMBER_OF_SERVOS_PER_LEG = 3


# simple range mapping function
def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


@dataclass
class Servo:
    """servo class to hold servo parameters"""
    id: int
    tty: str
    angle: int = 0
    pos: int = 0


@dataclass
class Leg:
    """leg class to hold servo positions"""
    id: int
    servos: List[Servo]


class ServoNotFoundError(EnvironmentError):
    pass


def get_servo_tty(servo_id, controllers):
    for tty, controller in controllers.items():
        try:
            controller.get_servo_id(servo_id, timeout=0.05)
            rospy.loginfo(f"found servo id {servo_id} on {tty}")
            return tty
        except TimeoutError:
            pass

    rospy.logerr(f"couldn't find a servo with ID: {servo_id} on any serial bus!")
    raise ServoNotFoundError


def set_tty_to_low_latency():
    rospy.loginfo("setting ports to low latency mode")

    for tty in TTY_LIST:
        os.system(f"setserial {tty} low_latency")


def spawn_reading_threads(servos, controllers):
    rospy.loginfo("spawning reading threads")

    thread_list = []
    for tty, controller in controllers.items():
        thread_list.append(ListenerThread([servo for servo in servos if servo.tty == tty], controller))
        thread_list[-1].start()

    return thread_list


class ListenerThread(Thread):
    def __init__(self, servos: List[Servo], controller: lewansoul_lx16a.ServoController):
        Thread.__init__(self)
        self.servos = servos
        self.controller = controller

    def run(self):
        while not rospy.is_shutdown():
            for servo in self.servos:
                try:
                    pos = self.controller.get_position(servo.id, timeout=0.02)
                    servo.pos = pos
                    servo.angle = round(fmap(pos, 0, 1000, -2.0944, 2.0944), 4)
                except TimeoutError:
                    rospy.logerr(f"couldn't read servo position. ID: {servo.id} port: {servo.tty}")
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
        angles = [servo.angle for leg in self.legs for servo in leg.servos]

        # yes
        self.joint_msg.position = [angles[6], angles[3], angles[0], angles[15], angles[12], angles[9],
                                   -angles[7], -angles[4], -angles[1], -angles[16], -angles[13], -angles[10],
                                   -angles[8], -angles[5], -angles[2], -angles[17], -angles[14], -angles[11], 0]

        self.joint_msg.header.stamp = rospy.Time.now()
        self.publisher.publish(self.joint_msg)

    def on_shutdown(self):
        rospy.loginfo(f"shutting down {type(self).__name__}")


if __name__ == '__main__':

    rospy.loginfo("starting hardware handler node")
    rospy.init_node('hardware_handler')

    # -------------------------------------------

    set_tty_to_low_latency()

    # -------------------------------------------

    rospy.loginfo("creating controller objects")
    controller_dict = {tty: lewansoul_lx16a.ServoController(serial.Serial(tty, 115200, timeout=1)) for tty in TTY_LIST}

    # -------------------------------------------

    rospy.loginfo("indexing servos")

    servo_list = [Servo(servo_id, get_servo_tty(servo_id, controller_dict)) for servo_id in range(1, NUMBER_OF_SERVOS + 1)]

    leg_list = [Leg(leg_id, servo_list[leg_id * 3:leg_id * 3 + 3]) for leg_id in range(NUMBER_OF_LEGS)]

    # -------------------------------------------

    spawn_reading_threads(servo_list, controller_dict)

    # -------------------------------------------

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
