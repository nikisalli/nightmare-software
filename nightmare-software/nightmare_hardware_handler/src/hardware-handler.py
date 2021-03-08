#!/usr/bin/env python3

import os
import time
from dataclasses import dataclass
from threading import Thread
from typing import List, Any

import lewansoul_lx16a
import rospy
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from nightmare_math.math import (fmap)

from nightmare_config.config import (PI,
                                     NUMBER_OF_LEGS,
                                     NUMBER_OF_SERVOS,
                                     TTY_LIST)


@dataclass
class Servo:
    """servo class to hold servo parameters"""
    id: int
    tty: str
    angle: int = 0
    pos: int = 0
    command: int = 0
    enabled: bool = False
    prev_enabled: bool = False


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
        except lewansoul_lx16a.TimeoutError:
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
        counter = 0
        while not rospy.is_shutdown():
            for i, servo in enumerate(self.servos):
                try:
                    # read servo
                    if(counter == i):
                        pos = self.controller.get_position(servo.id, timeout=0.02)
                        servo.pos = pos
                        servo.angle = round(fmap(pos, 0, 1000, -2.0944, 2.0944), 4)

                    # write servo
                    if servo.enabled and not servo.prev_enabled:
                        self.controller.motor_on(servo.id)
                        servo.prev_enabled = True
                    elif not servo.enabled and servo.prev_enabled:
                        self.controller.motor_off(servo.id)
                        servo.prev_enabled = False

                    if servo.enabled:
                        self.controller.move(servo.id, fmap(servo.command, -2.0944, 2.0944, 0, 1000))

                except lewansoul_lx16a.TimeoutError:
                    rospy.logerr(f"couldn't read servo position. ID: {servo.id} port: {servo.tty}")
            time.sleep(0.02)  # execute every 0.05s
            counter += 1
            if (counter > len(self.servos) * 2):  # multiplied by 2 so that only one read cycle over 2 is executed
                counter = 0


class HardwareHandlerNode:
    """main node class"""

    def __init__(self, legs: List[Leg], fixed_frame: Any, frame: Any, publisher: rospy.Publisher, joint_msg: JointState):
        self.legs = legs
        self.fixed_frame = fixed_frame
        self.frame = frame
        self.publisher = publisher
        self.joint_msg = joint_msg

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Ready for publishing")

    def publish(self):
        self._publish_jnt()

    def _publish_jnt(self):
        ang = [servo.angle for leg in self.legs for servo in leg.servos]

        self.joint_msg.position = [ang[0], -ang[1], -ang[2] - PI / 2,
                                   ang[3], -ang[4], -ang[5] - PI / 2,
                                   ang[6], -ang[7], -ang[8] - PI / 2,
                                   ang[9], -ang[10], -ang[11] - PI / 2,
                                   ang[12], -ang[13], -ang[14] - PI / 2,
                                   ang[15], -ang[16], -ang[17] - PI / 2, 0]

        self.joint_msg.header.stamp = rospy.Time.now()
        self.publisher.publish(self.joint_msg)

    def get_states(self, msg):
        for i, leg in enumerate(self.legs):
            leg.servos[0].enabled = msg.position[i * 3]
            leg.servos[1].enabled = msg.position[i * 3 + 1]
            leg.servos[2].enabled = msg.position[i * 3 + 2]

    def get_angles(self, msg):
        for i, leg in enumerate(self.legs):
            leg.servos[0].command = msg.position[i * 3]
            leg.servos[1].command = - msg.position[i * 3 + 1]
            leg.servos[2].command = - msg.position[i * 3 + 2] - PI / 2

    def on_shutdown(self):
        rospy.loginfo(f"shutting down {type(self).__name__}")


if __name__ == '__main__':

    rospy.loginfo("starting hardware handler node")
    rospy.init_node('hardware_handler')

    set_tty_to_low_latency()

    rospy.loginfo("creating controller objects")
    controller_dict = {tty: lewansoul_lx16a.ServoController(serial.Serial(tty, 115200, timeout=1)) for tty in TTY_LIST}

    rospy.loginfo("indexing servos")
    servo_list = [Servo(servo_id, get_servo_tty(servo_id, controller_dict)) for servo_id in range(1, NUMBER_OF_SERVOS)]
    leg_list = [Leg(leg_id, servo_list[leg_id * 3:leg_id * 3 + 3]) for leg_id in range(NUMBER_OF_LEGS)]

    spawn_reading_threads(servo_list, controller_dict)

    node = HardwareHandlerNode(
        legs=leg_list,
        fixed_frame=rospy.get_param('~fixed_frame', 'world'),  # set fixed frame relative to world to apply the transform
        frame=rospy.get_param('~frame', "base_link"),  # set frame name
        publisher=rospy.Publisher('joint_states', JointState, queue_size=10),  # joint state publisher
        joint_msg=JointState(header=Header(),
                             name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                   'leg2coxa', 'leg2femur', 'leg2tibia',
                                   'leg3coxa', 'leg3femur', 'leg3tibia',
                                   'leg4coxa', 'leg4femur', 'leg4tibia',
                                   'leg5coxa', 'leg5femur', 'leg5tibia',
                                   'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                             velocity=[],
                             effort=[])  # joint topic structure
    )

    rospy.Subscriber("/engine/angle_joint_states", JointState, node.get_angles)
    rospy.Subscriber("/engine/state_joint_states", JointState, node.get_states)

    try:
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            node.publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
