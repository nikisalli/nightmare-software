import lewansoul_lx16a
import serial
import numpy as np
import struct
import math

# ros imports
import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension
import tf_conversions

# every array is in the form of [[coxa, femur, tibia], [coxa, femur, tibia], ...] for every leg from 0 to 5 (6 legs)


# utility map function
def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class hardware_node:
    def __init__(self):
        # get serial port name from param server and create controller, if port unavailable keep trying forever
        while True or not rospy.is_shutdown():
            try:
                self.controller = lewansoul_lx16a.ServoController(serial.Serial(rospy.get_param("/hardware/servo_port"), 115200, timeout=0.1))
                self.sensor_port = serial.Serial(rospy.get_param("/hardware/sensor_port"), 115200, timeout=0.1)
                self.comm_port = serial.Serial(rospy.get_param("/hardware/comm_port"), 115200, timeout=0.1)
                break
            except Exception:
                rospy.logerr("Could not connect to servo controller")
                rospy.sleep(1)
        rospy.loginfo("all ports detected and connected")
        # this array can be used as a lookup table for the servo id's
        self.servo_index_to_id_map = np.array(rospy.get_param("/hardware/leg_configuration")).flatten()
        # self.counter = 0  # counter to read the servos less frequently
        self.hardware_angles = [0] * 18  # servo angles read from hardware
        self.commanded_angles = [0] * 18  # servo angles commanded by the engine
        self.commanded_enable = [False] * 18  # enable/disable servos
        self.already_enabled = [False] * 18  # enable/disable servo cache to avoid unnecessary enable/disable commands
        self.sensor_header = rospy.get_param("/hardware/sensor_header")

        # joint state publisher for robot_state_publisher to parse, this should publish joint angles read from hardware
        # this names come from the .trans file in nightmare_description and are used to drive the joints
        self.joint_msg = JointState(header=Header(),
                                    name=['leg1coxa', 'leg1femur', 'leg1tibia', 'leg2coxa', 'leg2femur', 'leg2tibia', 'leg3coxa', 'leg3femur', 'leg3tibia',
                                          'leg4coxa', 'leg4femur', 'leg4tibia', 'leg5coxa', 'leg5femur', 'leg5tibia', 'leg6coxa', 'leg6femur', 'leg6tibia'],
                                    velocity=[],
                                    effort=[])
        self.joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

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

        # subscriber to read the raw commands coming from the engine
        rospy.Subscriber("/engine/angle_joint_states", JointState, self.engine_angles_callback)
        rospy.Subscriber("/engine/enable_joint_states", JointState, self.engine_enables_callback)

        rospy.loginfo("hardware node ready")

    def engine_angles_callback(self, msg):
        # this is the raw angles from the engine
        self.commanded_angles = msg.position

    def engine_enables_callback(self, msg):
        # this is the enable/disable commands from the engine
        self.commanded_enable = int(msg.position)

    def update_servos(self):
        # write servos
        for index in range(18):
            if self.commanded_enable[index] is True:
                if self.already_enabled[index] is False:
                    self.controller.motor_on(self.servo_index_to_id_map[index])
                    self.already_enabled[index] = True
                self.controller.move(self.servo_index_to_id_map[index], fmap(self.commanded_angles[index], -2.0944, 2.0944, 0, 1000))
            else:
                self.controller.motor_off(self.servo_index_to_id_map[index])
                self.already_enabled[index] = False
                # read the servo only if it is disabled
                try:
                    self.hardware_angles[index] = fmap(self.controller.get_position(self.servo_index_to_id_map[index]), 0, 1000, -2.0944, 2.0944)
                except Exception:
                    rospy.logerr(f"could not read servo {self.servo_index_to_id_map[index]}")
        # publish joint state
        self.joint_msg.position = self.hardware_angles
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(self.joint_msg)

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

    rospy.loginfo("starting hardware node")

    node = hardware_node()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    rospy.loginfo("hardware node stopped")
