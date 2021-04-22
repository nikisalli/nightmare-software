#!/usr/bin/env python3

import os
import time
from dataclasses import dataclass
from threading import Thread
from typing import List, Any
import json
import math

import lewansoul_lx16a
import rospy
import serial
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header, Float32
import tf_conversions

from nightmare_math.math import fmap
from nightmare_config.config import (PI, NUMBER_OF_LEGS, NUMBER_OF_SERVOS, TTY_LIST, STAT_TTY, STAT_HEADER)


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
    t = time.time()
    while (time.time() - t < 30):
        for tty, controller in controllers.items():
            try:
                controller.get_servo_id(servo_id, timeout=0.05)
                rospy.loginfo(f"found servo id {servo_id} on {tty}")
                return tty
            except lewansoul_lx16a.TimeoutError:
                pass
        time.sleep(1)
        rospy.loginfo(f"couldn't find servo id {servo_id}. retrying...")

    rospy.logerr(f"couldn't find a servo with ID: {servo_id} on any serial bus!")
    raise ServoNotFoundError


def set_tty_to_low_latency():
    rospy.loginfo("setting ports to low latency mode")

    for tty in TTY_LIST:
        os.system(f"setserial {tty} low_latency")
    os.system(f"setserial {STAT_TTY} low_latency")


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
                    # read servo and write state less frequently (one servo per full servo iteration)
                    if(counter == i):
                        # read servo position
                        pos = self.controller.get_position(servo.id, timeout=0.02)
                        servo.pos = pos
                        servo.angle = round(fmap(pos, 0, 1000, -2.0944, 2.0944), 4)

                        # write servo state
                        if servo.enabled:
                            self.controller.motor_on(servo.id)
                        else:
                            self.controller.motor_off(servo.id)

                    if servo.enabled:
                        self.controller.move(servo.id, fmap(servo.command, -2.0944, 2.0944, 0, 1000))

                except lewansoul_lx16a.TimeoutError:
                    rospy.logerr(f"couldn't read servo position. ID: {servo.id} port: {servo.tty}")
            time.sleep(0.01)  # execute every 0.05s
            counter += 1
            if (counter > len(self.servos)):
                counter = 0


class statListenerThread(Thread):
    def __init__(self, node_stats: List[Any], imu: Any):
        Thread.__init__(self)
        self.stats = node_stats
        self.imu = imu
        self.port = serial.Serial(STAT_TTY, 921600, timeout=1)
        rospy.loginfo("stat thread started")

    def run(self):
        while not rospy.is_shutdown():
            header_buf = []
            # rolling header match
            while header_buf != STAT_HEADER:
                while self.port.in_waiting < 1:
                    time.sleep(0.1)
                header_buf.append(int.from_bytes(self.port.read(), "big"))
                if(len(header_buf) > len(STAT_HEADER)):
                    header_buf.pop(0)
            # read length
            length = int.from_bytes(self.port.read(2), "big")
            while self.port.in_waiting < length:
                time.sleep(0.1)
            # read packet id
            packet_id = int.from_bytes(self.port.read(), "big")
            # read data chunk
            data_chunk = self.port.read(length - 1)
            if(packet_id == 0):
                data = json.loads(data_chunk)
                self.stats[0] = max(0, data['c1'])  # 0 if negative
                self.stats[1] = data['c']
                self.stats[2] = max(0, data['v'])
                self.imu[1] = data['R'] / -100.  # the imu sends data as degrees * 100 so we scale it down
                self.imu[2] = data['P'] / -100.
                self.imu[0] = data['Y'] / 100.


class HardwareHandlerNode:
    """main node class"""

    def __init__(self, legs: List[Leg], publisher: rospy.Publisher, joint_msg: JointState, node_stats: List[Any], imu: Any):
        self.legs = legs
        self.publisher = publisher
        self.joint_msg = joint_msg
        self.stats = node_stats
        self.imu = imu
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration_covariance[0] = -1  # we don't have acc data
        self.imu_msg.angular_velocity_covariance[0] = -1  # we don't have gyro data
        self.imu_msg.orientation_covariance[0] = 0.00017  # imu orientation covariance
        self.imu_msg.orientation_covariance[4] = 0.00017
        self.imu_msg.orientation_covariance[8] = 0.00017
        self.imu_msg.header.frame_id = 'body_imu'  # set imu frame
        self.publisher_servo_current = rospy.Publisher('servo_current', Float32, queue_size=10)
        self.publisher_computer_current = rospy.Publisher('computer_current', Float32, queue_size=10)
        self.publisher_voltage = rospy.Publisher('voltage', Float32, queue_size=10)
        self.publisher_current = rospy.Publisher('current', Float32, queue_size=10)
        self.publisher_imu = rospy.Publisher('body_imu', Imu, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Ready for publishing")

    def publish(self):
        self._publish_jnt()
        self._publish_stat()
        self._publish_imu()

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

    def _publish_stat(self):
        self.publisher_servo_current.publish(self.stats[0])
        self.publisher_computer_current.publish(self.stats[1])
        self.publisher_voltage.publish(self.stats[2])
        self.publisher_current.publish(self.stats[0] + self.stats[1])

    def _publish_imu(self):
        self.imu_msg.header.stamp = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(math.radians(self.imu[0]),
                                                                 math.radians(self.imu[1]),
                                                                 math.radians(self.imu[2]))
        self.imu_msg.orientation.w = q[0]
        self.imu_msg.orientation.x = q[1]
        self.imu_msg.orientation.y = q[2]
        self.imu_msg.orientation.z = q[3]
        self.publisher_imu.publish(self.imu_msg)

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

    stats = [0, 0, 0]
    imu_rpy = [0, 0, 0]

    stat_thread = statListenerThread(stats, imu_rpy)
    stat_thread.start()

    node = HardwareHandlerNode(
        legs=leg_list,
        publisher=rospy.Publisher('joint_states', JointState, queue_size=10),  # joint state publisher
        joint_msg=JointState(header=Header(),
                             name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                   'leg2coxa', 'leg2femur', 'leg2tibia',
                                   'leg3coxa', 'leg3femur', 'leg3tibia',
                                   'leg4coxa', 'leg4femur', 'leg4tibia',
                                   'leg5coxa', 'leg5femur', 'leg5tibia',
                                   'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                             velocity=[],
                             effort=[]),  # joint topic structure
        node_stats=stats,
        imu=imu_rpy
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
