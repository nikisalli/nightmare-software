#!/usr/bin/env python3
# pylint: disable=broad-except, no-name-in-module

import os
import time
from dataclasses import dataclass, field
from threading import Thread
from typing import List, Any
import math
import numpy as np

import lewansoul_lx16a
import rospy
import serial
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension
import tf_conversions

from nightmare_math.math import fmap
from nightmare_config.config import (PI, NUMBER_OF_LEGS, NUMBER_OF_SERVOS, SERVO_TTY, STAT_TTY, STAT_HEADER)


@dataclass
class Servo:
    id: int
    angle: int = 0
    pos: int = 0
    command: int = 0
    enabled: bool = False
    controller: lewawa_lx16a.ServoController() = field(init=False)


@dataclass
class Leg:
    id: int
    servos: List[Servo]
    force: float = 0.0


class HardwareHandlerNode:
    """main node class"""

    def __init__(self, publisher: rospy.Publisher, joint_msg: JointState):
        # servo controller
        servo_controller = lewansoul_lx16a.ServoController(serial.Serial(SERVO_TTY, 115200, timeout=1))

        servo_list = [Servo(servo_id) for servo_id in range(1, NUMBER_OF_SERVOS)]
        self.legs = [Leg(leg_id, servo_list[leg_id * 3:leg_id * 3 + 3]) for leg_id in range(NUMBER_OF_LEGS)]

        self.counter = 0  # counter to read the servos less frequently
        self.joint_msg = JointState(header=Header(),
                                    name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                          'leg2coxa', 'leg2femur', 'leg2tibia',
                                          'leg3coxa', 'leg3femur', 'leg3tibia',
                                          'leg4coxa', 'leg4femur', 'leg4tibia',
                                          'leg5coxa', 'leg5femur', 'leg5tibia',
                                          'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                                    velocity=[],
                                    effort=[])

        # imu
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration_covariance[0] = -1  # we don't have acc data
        self.imu_msg.angular_velocity_covariance[0] = -1  # we don't have gyro data
        self.imu_msg.orientation_covariance[0] = 0.00017  # imu orientation covariance
        self.imu_msg.orientation_covariance[4] = 0.00017
        self.imu_msg.orientation_covariance[8] = 0.00017
        self.imu_msg.header.frame_id = 'body_imu'  # set imu frame

        # load cells
        self.load_cell_msg = Float32MultiArray()
        self.load_cell_msg.layout.dim.append(MultiArrayDimension())
        self.load_cell_msg.layout.dim[0].label = "height"
        self.load_cell_msg.layout.dim[0].size = 6
        self.load_cell_msg.layout.dim[0].stride = 6
        self.load_cell_msg.layout.data_offset = 0
        self.load_cell_msg.data = np.asarray([0.] * 6)

        # publishers
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.publisher_voltage = rospy.Publisher('voltage', Float32, queue_size=10)
        self.publisher_current = rospy.Publisher('current', Float32, queue_size=10)
        self.publisher_imu = rospy.Publisher('body_imu', Imu, queue_size=10)
        self.publisher_load_cells = rospy.Publisher('load_cells', Float32MultiArray, queue_size=10)

        # subscribers
        rospy.Subscriber("/engine/angle_joint_states", JointState, self.get_angles)
        rospy.Subscriber("/engine/state_joint_states", JointState, self.get_states)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Ready for publishing")

    def run(run):
        self.handle_servos()

        self._publish_jnt()
        self._publish_stat()
        self._publish_imu()
        self._publish_sensors()

    def handle_servos(self):
        for i, servo in enumerate(self.servos):
            try:
                # read servo and write state less frequently (one servo per full servo iteration)
                if(self.counter == i):
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
        self.counter += 1

    def handle_hw_stats(self):
        buf = []
        while buf != STAT_HEADER:
            buf.append(int.from_bytes(ser.read(), "big"))
            buf = buf[-4:]
        buf = []
        checksum = 0
        for i in range(72):
            buf.append(ser.read())
            checksum += int.from_bytes(buf[-1], "big")
        checksum = checksum % 256
        rchecksum = int.from_bytes(ser.read(), "big")
        data = None
        if checksum == rchecksum:
            data = struct.unpack('HfHfiiiiiiHffHHHfff', b''.join(buf))
            print(data)

        # read data chunk
        # data_chunk = self.port.read(length - 1)
        # if(packet_id == 0):
        #     data = json.loads(data_chunk)
        #     self.stats[0] = max(0, data['c1'])  # 0 if negative
        #     self.stats[1] = data['c']
        #     self.stats[2] = max(0, data['v'])
        #     self.imu[1] = data['R'] / -100.  # the imu sends data as degrees * 100 so we scale it down
        #     self.imu[2] = data['P'] / -100.
        #     self.imu[0] = data['Y'] / 100.

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
        q = tf_conversions.transformations.quaternion_from_euler(math.radians(self.imu[0]), math.radians(self.imu[1]), math.radians(self.imu[2]))
        self.imu_msg.orientation.w = q[0]
        self.imu_msg.orientation.x = q[1]
        self.imu_msg.orientation.y = q[2]
        self.imu_msg.orientation.z = q[3]
        self.publisher_imu.publish(self.imu_msg)

    def _publish_sensors(self):
        self.sensor_msg_raw = [leg.force_sensor.force for leg in self.legs]
        self.sensor_msg.data = self.sensor_msg_raw
        self.filtered_sensor_msg.data += FORCE_SENSOR_FILTER_VAL * (self.sensor_msg_raw - self.filtered_sensor_msg.data)
        self.publisher_force_sensors.publish(self.sensor_msg)
        self.publisher_filtered_force_sensors.publish(self.filtered_sensor_msg)

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

    node = HardwareHandlerNode()

    try:
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            node.publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
