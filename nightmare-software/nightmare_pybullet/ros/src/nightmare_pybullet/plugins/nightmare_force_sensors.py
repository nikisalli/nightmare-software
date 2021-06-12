#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension
import numpy as np

FILTER_VAL = 0.99


class nightmareForceSensors:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        self.pb.setRealTimeSimulation(1)

        # get robot from parent class
        self.robot = robot
        self.tibias = {}
        self.index_tibias()
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

        self.publisher_force_sensors = rospy.Publisher('force_sensors', Float32MultiArray, queue_size=10)
        self.publisher_filtered_force_sensors = rospy.Publisher('filtered_force_sensors', Float32MultiArray, queue_size=10)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        points = self.pb.getContactPoints(self.robot)
        for p in points:
            _id = p[3]
            if _id in self.tibias:
                self.forces[self.tibias[p[3]]] = round(p[9] * 10., 2)
        self.sensor_msg.data = self.forces
        self.filtered_sensor_msg.data = FILTER_VAL * self.prev_forces + (1. - FILTER_VAL) * self.forces  # simple first order low pass filter
        self.prev_filtered_forces = self.filtered_sensor_msg.data
        self.publisher_force_sensors.publish(self.sensor_msg)
        self.publisher_filtered_force_sensors.publish(self.filtered_sensor_msg)

    def index_tibias(self):
        for i in range(self.pb.getNumJoints(self.robot)):
            n = self.pb.getJointInfo(self.robot, i)[12].decode('UTF-8')
            if "tibia" in n:
                # fill the list with the id of the tibia of leg n at place n - 1 (leg count starts at 1)
                self.tibias[i] = int(n[4]) - 1
