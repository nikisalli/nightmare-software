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

# every array is in the form of [[coxa, femur, tibia], [coxa, femur, tibia], ...] for every leg from 0 to 5 (6 legs)


# utility functions
def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class hardware_node:
    def __init__(self):
        # create controller, if port unavailable keep trying forever
        while True or not rospy.is_shutdown():
            try:
                self.controller = lewansoul_lx16a.ServoController(serial.Serial(SERVO_PORT, 115200, timeout=0.1))
                self.sensor_port = serial.Serial(SENSOR_PORT, 115200, timeout=0.1)
                self.comm_port = serial.Serial(COMMUNICATION_PORT, 115200, timeout=0.1)
                break
            except Exception:
                perr("Could not connect to servo controller")
                rospy.sleep(1)
        pinfo("all ports detected and connected")
        # this array can be used as a lookup table for the servo id's
        self.servo_index_to_id_map = np.array(SERVO_IDS)
        # self.counter = 0  # counter to read the servos less frequently
        self.hardware_angles = [0] * 18  # servo angles read from hardware
        self.commanded_angles = [0] * 18  # servo angles commanded by the engine
        self.commanded_enable = [False] * 18  # enable/disable servos
        self.already_enabled = [False] * 18  # enable/disable servo cache to avoid unnecessary enable/disable commands
        self.sensor_header = SENSOR_HEADER

        # ######## PUBLISHERS ########
        # joint state publisher for robot_state_publisher to parse, this should publish joint angles read from hardware
        # this names come from the .trans file in nightmare_description and are used to drive the joints

        self.hardware_joint_publisher_msg = JointState(header=Header(), name=JOINT_STATE_LABELS)
        self.hardware_joint_publisher = rospy.Publisher('/hardware/joint_states', JointState, queue_size=10)
        self.urdf_joint_publisher_msg = JointState(header=Header(), name=JOINT_STATE_LABELS)
        self.urdf_joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # imu
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration_covariance[0] = -1  # we don't have acc data
        self.imu_msg.angular_velocity_covariance[0] = -1  # we don't have gyro data
        self.imu_msg.orientation_covariance[0] = 0.00017  # imu orientation covariance
        self.imu_msg.orientation_covariance[4] = 0.00017
        self.imu_msg.orientation_covariance[8] = 0.00017
        self.imu_msg.header.frame_id = '/body_imu'  # reference frame for the imu in the transformation tree
        self.imu_publisher = rospy.Publisher('/hardware/body_imu', Imu, queue_size=10)

        # load cell
        self.load_cell_msg = Float32MultiArray()
        self.load_cell_msg.layout.dim.append(MultiArrayDimension())
        self.load_cell_msg.layout.dim[0].label = "height"
        self.load_cell_msg.layout.dim[0].size = 6
        self.load_cell_msg.layout.dim[0].stride = 6
        self.load_cell_msg.layout.data_offset = 0
        self.load_cell_msg.data = np.asarray([0.] * 6)
        self.load_cell_publisher = rospy.Publisher('/hardware/load_cells', Float32MultiArray, queue_size=10)

        # power
        self.voltage_publisher = rospy.Publisher('/hardware/battery/voltage', Float32, queue_size=10)
        self.current_publisher = rospy.Publisher('/hardware/battery/current', Float32, queue_size=10)

        # ######## SUBSCRIBERS ########
        # subscriber to read the raw commands coming from the engine
        rospy.Subscriber("/engine/joint_states", JointState, self.engine_angles_callback)

        pinfo("ready")

    def engine_angles_callback(self, msg):
        # this is the raw angles from the engine
        self.commanded_angles = msg.position
        self.commanded_enable = [bool(x) for x in msg.effort]

    def enable_motor(self, index):
        sid = self.servo_index_to_id_map[index]
        if self.already_enabled[index] is False:
            self.controller.motor_on(sid)
            # wait for feedback
            try:
                if self.controller.is_motor_on(sid, timeout=0.1):
                    self.already_enabled[index] = True
                else:
                    perr(f"could not enable servo id: {sid}")
            except Exception as e:
                perr(f"could not enable servo id: {sid} exception: {type(e).__name__}")

    def disable_motor(self, index):
        sid = self.servo_index_to_id_map[index]
        if self.already_enabled[index] is True:
            self.controller.motor_off(sid)
            # wait for feedback
            try:
                if not self.controller.is_motor_on(sid, timeout=0.1):
                    self.already_enabled[index] = False
                else:
                    perr(f"could not disable servo id: {sid}")
            except Exception as e:
                perr(f"could not disable servo id: {sid} exception: {type(e).__name__}")

    def update_servos(self):
        # write servos
        for index in range(18):
            sid = self.servo_index_to_id_map[index]
            if self.commanded_enable[index] is True:
                self.enable_motor(index)
                self.controller.move(sid, fmap(self.commanded_angles[index], -2.0944, 2.0944, 0, 1000))
            else:
                self.disable_motor(index)
                try:
                    self.hardware_angles[index] = fmap(self.controller.get_position(sid, timeout=0.1), 0, 1000, -2.0944, 2.0944)
                except Exception as e:
                    perr(f"could not read servo {sid} exception: {type(e).__name__}")
        # publish hardware joint state
        # these are the real angles read from the hardware
        self.hardware_joint_publisher_msg.position = self.hardware_angles
        self.hardware_joint_publisher_msg.effort = [bool(x) for x in self.already_enabled]
        self.hardware_joint_publisher_msg.header.stamp = rospy.Time.now()
        self.hardware_joint_publisher.publish(self.hardware_joint_publisher_msg)
        # publish urdf joint state
        # these are offset angles to accomodate for any discrepancies between the hardware and the urdf
        self.urdf_joint_publisher_msg.position = [angle + offset for angle, offset in zip(self.hardware_angles, URDF_JOINT_OFFSETS)]
        self.urdf_joint_publisher_msg.effort = [bool(x) for x in self.already_enabled]
        self.urdf_joint_publisher_msg.header.stamp = rospy.Time.now()
        self.urdf_joint_publisher.publish(self.urdf_joint_publisher_msg)

    def parse_sensors(self):
        # flush the serial buffer to avoid reading old data
        self.sensor_port.read(self.sensor_port.in_waiting - 120)
        buf = []
        while buf != [0x55, 0x55, 0x55, 0x55]:
            buf.append(int.from_bytes(self.sensor_port.read(), "big"))
            buf = buf[-4:]
        buf = []
        checksum = 0
        for i in range(72):
            buf.append(self.sensor_port.read())
            checksum += int.from_bytes(buf[-1], "big")
        checksum = checksum % 256
        rchecksum = int.from_bytes(self.sensor_port.read(), "big")
        data = None
        if checksum == rchecksum:
            data = struct.unpack('HfHfiiiiiiHffHHHfff', b''.join(buf))

            # publish imu data
            self.imu_msg.header.stamp = rospy.Time.now()
            q = tf_conversions.transformations.quaternion_from_euler(math.radians(data[16]), math.radians(data[17]), math.radians(data[18]))
            self.imu_msg.orientation.w = q[0]
            self.imu_msg.orientation.x = q[1]
            self.imu_msg.orientation.y = q[2]
            self.imu_msg.orientation.z = q[3]
            self.imu_publisher.publish(self.imu_msg)

            # publish load cell data
            self.load_cell_msg.data = np.asarray(data[4:9])
            self.load_cell_publisher.publish(self.load_cell_msg)

            # publish voltage and current
            self.voltage_publisher.publish(data[1])
            self.current_publisher.publish(data[3])
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

    node = hardware_node()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    pinfo("stopped")
