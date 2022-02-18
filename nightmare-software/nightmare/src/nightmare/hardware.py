import lewansoul_lx16a
import serial
import numpy as np
import struct
import math
import sys

# ros imports
import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension
import tf_conversions

# module imports
from nightmare.config import *
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal
from nightmare.modules.math import fmap

# every array is in the form of [[coxa, femur, tibia], [coxa, femur, tibia], ...] for every leg from 0 to 5 (6 legs)


class HardwareNode:
    def __init__(self):
        # create controller, if port unavailable keep trying forever
        while True or not rospy.is_shutdown():
            try:
                self._controller = lewansoul_lx16a.ServoController(serial.Serial(SERVO_PORT, 115200, timeout=0.1))
                self._sensor_port = serial.Serial(SENSOR_PORT, 115200, timeout=0.1)
                self._comm_port = serial.Serial(COMMUNICATION_PORT, 115200, timeout=0.1)
                break
            except Exception:
                perr("Could not connect to servo controller")
                rospy.sleep(1)
        pinfo("all ports detected and connected")
        # this array can be used as a lookup table for the servo id's
        self._servo_index_to_id_map = np.array(SERVO_IDS)
        # self._counter = 0  # counter to read the servos less frequently
        self._hardware_angles = [0] * 18  # servo angles read from hardware
        self._commanded_angles = [0] * 18  # servo angles commanded by the engine
        self._commanded_enable = [False] * 18  # enable/disable servos
        self._already_enabled = [False] * 18  # enable/disable servo cache to avoid unnecessary enable/disable commands
        self._sensor_header = SENSOR_HEADER

        # ######## PUBLISHERS ########
        # joint state publisher for robot_state_publisher to parse, this should publish joint angles read from hardware
        # this names come from the .trans file in nightmare_description and are used to drive the joints

        self._hardware_joint_publisher_msg = JointState(header=Header(), name=JOINT_STATE_LABELS)
        self._hardware_joint_publisher = rospy.Publisher('/hardware/joint_states', JointState, queue_size=10)
        self._urdf_joint_publisher_msg = JointState(header=Header(), name=JOINT_STATE_LABELS)
        self._urdf_joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # imu
        self._imu_msg = Imu()
        self._imu_msg.linear_acceleration_covariance[0] = -1  # we don't have acc data
        self._imu_msg.angular_velocity_covariance[0] = -1  # we don't have gyro data
        self._imu_msg.orientation_covariance[0] = 0.00017  # imu orientation covariance
        self._imu_msg.orientation_covariance[4] = 0.00017
        self._imu_msg.orientation_covariance[8] = 0.00017
        self._imu_msg.header.frame_id = '/body_imu'  # reference frame for the imu in the transformation tree
        self._imu_publisher = rospy.Publisher('/hardware/body_imu', Imu, queue_size=10)

        # load cell
        self._load_cell_calibration_data = np.zeros(6)
        self._load_cell_msg = Float32MultiArray()
        self._load_cell_msg.layout.dim.append(MultiArrayDimension())
        self._load_cell_msg.layout.dim[0].label = "height"
        self._load_cell_msg.layout.dim[0].size = 6
        self._load_cell_msg.layout.dim[0].stride = 6
        self._load_cell_msg.layout.data_offset = 0
        self._load_cell_msg.data = np.asarray([0.] * 6)
        self._load_cell_publisher = rospy.Publisher('/hardware/load_cells', Float32MultiArray, queue_size=10)

        # power
        self._voltage_publisher = rospy.Publisher('/hardware/battery/voltage', Float32, queue_size=10)
        self._current_publisher = rospy.Publisher('/hardware/battery/current', Float32, queue_size=10)

        # ######## SUBSCRIBERS ########
        # subscriber to read the raw commands coming from the engine
        rospy.Subscriber("/engine/joint_states", JointState, self.engine_angles_callback)

        pinfo("ready")

    def engine_angles_callback(self, msg: JointState):
        '''get pose from engine'''
        self._commanded_angles = msg.position
        self._commanded_enable = [bool(x) for x in msg.effort]

    def enable_motor(self, index: int):
        sid = self._servo_index_to_id_map[index]
        if self._already_enabled[index] is False:
            self._controller.motor_on(sid)
            # wait for feedback
            try:
                if self._controller.is_motor_on(sid, timeout=0.1):
                    self._already_enabled[index] = True
                else:
                    perr(f"could not enable servo id: {sid}")
            except Exception as e:
                perr(f"could not enable servo id: {sid} exception: {type(e).__name__}")

    def disable_motor(self, index: int):
        sid = self._servo_index_to_id_map[index]
        if self._already_enabled[index] is True:
            self._controller.motor_off(sid)
            # wait for feedback
            try:
                if not self._controller.is_motor_on(sid, timeout=0.1):
                    self._already_enabled[index] = False
                else:
                    perr(f"could not disable servo id: {sid}")
            except Exception as e:
                perr(f"could not disable servo id: {sid} exception: {type(e).__name__}")

    def update_servos(self):
        '''write and read servos'''
        for index in range(18):
            sid = self._servo_index_to_id_map[index]
            if self._commanded_enable[index] is True:
                self.enable_motor(index)
                self._controller.move(sid, fmap(self._commanded_angles[index], -2.0944, 2.0944, 0, 1000))
                # if everything is ok the servo will reach the commanded angle immediately so we can save bandwidth on the bus by assuming it did
                self._hardware_angles[index] = self._commanded_angles[index]
            else:
                self.disable_motor(index)
                try:
                    self._hardware_angles[index] = fmap(self._controller.get_position(sid, timeout=0.1), 0, 1000, -2.0944, 2.0944)
                except Exception as e:
                    perr(f"could not read servo {sid} exception: {type(e).__name__}")
        # publish hardware joint state
        # these are the real angles read from the hardware
        self._hardware_joint_publisher_msg.position = self._hardware_angles
        self._hardware_joint_publisher_msg.effort = [bool(x) for x in self._already_enabled]
        self._hardware_joint_publisher_msg.header.stamp = rospy.Time.now()
        self._hardware_joint_publisher.publish(self._hardware_joint_publisher_msg)
        # publish urdf joint state
        # add offset REAL -> URDF
        self._urdf_joint_publisher_msg.position = np.array(self._hardware_angles) + URDF_JOINT_OFFSETS
        self._urdf_joint_publisher_msg.effort = [bool(x) for x in self._already_enabled]
        self._urdf_joint_publisher_msg.header.stamp = rospy.Time.now()
        self._urdf_joint_publisher.publish(self._urdf_joint_publisher_msg)

    def parse_sensors(self):
        # flush the serial buffer to avoid reading old data
        self._sensor_port.read(self._sensor_port.in_waiting - 120)
        buf = []
        while buf != [0x55, 0x55, 0x55, 0x55]:
            buf.append(int.from_bytes(self._sensor_port.read(), "big"))
            buf = buf[-4:]
        buf = []
        checksum = 0
        for i in range(72):
            buf.append(self._sensor_port.read())
            checksum += int.from_bytes(buf[-1], "big")
        checksum = checksum % 256
        rchecksum = int.from_bytes(self._sensor_port.read(), "big")
        data = None
        if checksum == rchecksum:
            data = struct.unpack('HfHfiiiiiiHffHHHfff', b''.join(buf))

            # publish imu data
            self._imu_msg.header.stamp = rospy.Time.now()
            q = tf_conversions.transformations.quaternion_from_euler(math.radians(data[16]), math.radians(data[17]), math.radians(data[18]))
            self._imu_msg.orientation.w = q[0]
            self._imu_msg.orientation.x = q[1]
            self._imu_msg.orientation.y = q[2]
            self._imu_msg.orientation.z = q[3]
            self._imu_publisher.publish(self._imu_msg)

            # publish load cell data
            try:
                self._load_cell_calibration_data = np.array(rospy.get_param("/hardware/load_cells_offsets"))
            except KeyError:
                pass
            self._load_cell_msg.data = (np.array([data[9], data[5], data[6], data[8], data[7], data[4]]) / LOAD_CELLS_CONVERSION_FACTOR) - self._load_cell_calibration_data
            self._load_cell_publisher.publish(self._load_cell_msg)

            # publish voltage and current
            self._voltage_publisher.publish(data[1])
            self._current_publisher.publish(data[3])
        else:
            # rospy.logerr(f"sensor checksum error {checksum} != {rchecksum}")
            pass

    def update(self):
        # publish everything
        self.update_servos()
        self.parse_sensors()


if __name__ == '__main__':
    rospy.init_node('hardware_handler')

    pinfo("starting")

    node = HardwareNode()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    pinfo("stopped")
