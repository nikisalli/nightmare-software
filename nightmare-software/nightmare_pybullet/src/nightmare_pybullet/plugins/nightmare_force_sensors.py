#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension
import numpy as np


URDF_JOINT_OFFSETS = np.array([0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854, 0, -1.2734, -0.7854])
JOINT_STATE_LABELS = ['leg1coxa', 'leg1femur', 'leg1tibia',
                      'leg2coxa', 'leg2femur', 'leg2tibia',
                      'leg3coxa', 'leg3femur', 'leg3tibia',
                      'leg4coxa', 'leg4femur', 'leg4tibia',
                      'leg5coxa', 'leg5femur', 'leg5tibia',
                      'leg6coxa', 'leg6femur', 'leg6tibia']
URDF_DICT = {name: {'offset': offset} for name in JOINT_STATE_LABELS for offset in URDF_JOINT_OFFSETS}
FILTER_VAL = 0.99


class nightmareForceSensors:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        self.pb.setRealTimeSimulation(1)

        # get robot from parent class
        self.robot = robot
        self.forces = np.array([0.] * 6)
        self.prev_forces = np.array([0.] * 6)

        self.sensor_msg = Float32MultiArray()
        self.sensor_msg.layout.dim.append(MultiArrayDimension())
        self.sensor_msg.layout.dim[0].label = "height"
        self.sensor_msg.layout.dim[0].size = 6
        self.sensor_msg.layout.dim[0].stride = 6
        self.sensor_msg.layout.data_offset = 0
        self.sensor_msg.data = np.asarray([0.] * 6)
        self.sensor_msg_raw = np.asarray([0.] * 6)

        self.filtered_sensor_msg = Float32MultiArray()
        self.filtered_sensor_msg.layout.dim.append(MultiArrayDimension())
        self.filtered_sensor_msg.layout.dim[0].label = "height"
        self.filtered_sensor_msg.layout.dim[0].size = 6
        self.filtered_sensor_msg.layout.dim[0].stride = 6
        self.filtered_sensor_msg.layout.data_offset = 0
        self.filtered_sensor_msg.data = np.asarray([0.] * 6)
        self.filtered_sensor_msg_raw = np.asarray([0.] * 6)

        # self.publisher_force_sensors = rospy.Publisher('force_sensors', Float32MultiArray, queue_size=10)
        self.publisher_filtered_force_sensors = rospy.Publisher('/hardware/load_cells', Float32MultiArray, queue_size=10)

        for i in range(self.pb.getNumJoints(self.robot)):
            self.pb.enableJointForceTorqueSensor(self.robot, i, 1)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # create arrays as long as the number of joints to store positions and torques
        self.positions = np.array([0.] * self.pb.getNumJoints(self.robot))
        self.torques = np.array([0.] * self.pb.getNumJoints(self.robot))
        # fill the arrays
        for i in range(self.pb.getNumJoints(self.robot)):
            name = self.pb.getJointInfo(self.robot, i)[1].decode("utf-8")
            if 'femur' in name:
                leg_num = int(name[3]) - 1
                self.forces[leg_num] = self.pb.getJointState(self.robot, i)[2][2] / 9.81

        # publish
        self.sensor_msg.data = self.forces
        self.filtered_sensor_msg.data = FILTER_VAL * self.prev_forces + (1. - FILTER_VAL) * self.forces  # simple first order low pass filter
        self.prev_filtered_forces = self.filtered_sensor_msg.data
        # self.publisher_force_sensors.publish(self.sensor_msg)
        self.publisher_filtered_force_sensors.publish(self.filtered_sensor_msg)
