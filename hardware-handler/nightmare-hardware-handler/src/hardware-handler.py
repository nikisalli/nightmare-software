#!/usr/bin/env python

import rospy
import serial
import numpy as np
import tf
import math
import time

from threading import Thread
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster

robot = serial.Serial('/dev/ttyUSB0', 460800)                                                      #robot serial port
imu = serial.Serial('/dev/ttyACM0', 115200)                                                        #imu serial port

robot.isOpen()

def fmap(x, in_min, in_max, out_min, out_max):                                                     #simple linear interpolation function
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min

class robot_listener(Thread):                                                                      #robot listener to parse data coming from the robot
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while 1:
            if(ord(robot.read()) == 0xAA):                                                         #search for first header byte
                if(ord(robot.read()) == 0xBB):                                                     #search for second header byte
                    bytes = []
                    for _ in range(26): 
                        bytes.append(ord(robot.read()))                                            #one message is made of 26 bytes so read them all
                    robot_listener.voltage = fmap(bytes[0],0,255,5,10)                             #scale variables to readable data
                    robot_listener.current = fmap(bytes[1],0,255,0,15)
                    robot_listener.joints = bytes[2:26]                                            #the last 24 bytes of the message are servo angles

class imu_listener(Thread):                                                                        #imu listener to parse imu data
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while 1:
            if(ord(imu.read()) == 0xBB):                                                           #search for the header
                imu.read()
                bytes = []
                for _ in range(3):
                    bytes.append(ord(imu.read()))
                imu_listener.roll = fmap(bytes[0],0,255,-180,180)                                  #scale bytes to readable data
                imu_listener.pitch = fmap(bytes[1],0,255,-180,180)
                imu_listener.yaw = fmap(bytes[2],0,255,-180,180)

class nightmare_node():
    def __init__(self):
        self.degrees2rad = math.pi/180.0

        # Get values from launch file

        self.rate = rospy.get_param('~rate', 100.0)                                                #the rate at which to publish the transform

        
        self.static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])           #static transform between sensor and fixed frame: r, p, y, x, y, z
        self.topic_name = rospy.get_param('~topic_name', "/imu")                                   #set imu topic name
        self.fixed_frame = rospy.get_param('~fixed_frame', "world")                                #set fixed frame to apply the transform
        self.frame_name = rospy.get_param('~frame_name', "imu")                                    #set frame name
        self.publish_transform = rospy.get_param('~publish_transform', False)                      #set true in lunch file if the transform is needed

        self.pub_imu = rospy.Publisher("nightmare/imu", Imu, queue_size=1)                         #create topics name, type and data persistance
        self.pub_vol = rospy.Publisher("nightmare/battery/voltage", Float32, queue_size=1)
        self.pub_cur = rospy.Publisher("nightmare/battery/current", Float32, queue_size=1)
        self.pub_jnt = rospy.Publisher("nightmare/joint_states", JointState, queue_size=10)

        self.odomBroadcaster_imu = TransformBroadcaster()
        self.imu_msg = Imu()

        self.jnt_msg = JointState()
        self.jnt_msg.header = Header()
        self.jnt_msg.name = ['Rev24', 'Rev25', 'Rev26', 'Rev27', 'Rev28', 'Rev29', 'Rev30', 'Rev31', 'Rev32', 
                             'Rev33', 'Rev34', 'Rev35', 'Rev36', 'Rev37', 'Rev38', 'Rev39', 'Rev48', 'Rev49', 
                             'Rev50', 'Rev51', 'Rev52', 'Rev53', 'Rev54', 'Rev55', 'Rev65', 'Rev66', 'Rev69']
        self.jnt_msg.velocity = []
        self.jnt_msg.effort = []

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(50) # 100hz

        rospy.loginfo("Ready for publishing")

        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            if self.publish_transform:
                quaternion = tf.transformations.quaternion_from_euler(self.static_transform[3]*self.degrees2rad,
                                                                      self.static_transform[4]*self.degrees2rad,
                                                                      self.static_transform[5]*self.degrees2rad)

                self.odomBroadcaster_imu.sendTransform(
                    (self.static_transform[0], self.static_transform[1], self.static_transform[2]),
                    (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                    rospy.Time.now(), self.frame_name, self.fixed_frame
                )
            
            self.publish_imu()
            self.publish_vol()
            self.publish_cur()
            self.publish_jnt()

            rate.sleep()

    def publish_imu(self):
        self.imu_msg = Imu()

        self.imu_msg.orientation_covariance = [0.01 ,0    ,0
                                              ,0    ,0.01 ,0
                                              ,0    ,0    ,0.01]

        quaternion = tf.transformations.quaternion_from_euler(imu_listener.roll*self.degrees2rad,
                                                              imu_listener.pitch*self.degrees2rad,
                                                              imu_listener.yaw*self.degrees2rad)

        self.imu_msg.orientation.x = quaternion[0] # x
        self.imu_msg.orientation.y = quaternion[1] # y
        self.imu_msg.orientation.z = quaternion[2] # z
        self.imu_msg.orientation.w = quaternion[3] # w

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.frame_name
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)
        self.seq += 1

    def publish_vol(self):
        self.pub_vol.publish(robot_listener.voltage)

    def publish_cur(self):
        self.pub_cur.publish(robot_listener.current)

    def publish_jnt(self):
        res = robot_listener.joints
        tmp = [fmap(i,0,255,180,-180)*self.degrees2rad for i in res] 

        self.jnt_msg.position = [-tmp[12], -tmp[9], -tmp[6], -tmp[3], -tmp[0], -tmp[21], -tmp[18], -tmp[15], tmp[10],
                                 tmp[7], tmp[4], tmp[1], tmp[13], tmp[16], tmp[19], tmp[22], tmp[14], tmp[17],
                                 tmp[20], tmp[23], tmp[2], tmp[5], tmp[8], tmp[11], 0, 0, 0]

        self.jnt_msg.header.stamp = rospy.Time.now()
        self.pub_jnt.publish(self.jnt_msg)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

if __name__ == '__main__':
    robot_listener()
    imu_listener()

    rospy.loginfo('Starting RobotImuPublisherNode')
    rospy.init_node('sensor_imu_publisher')

    try:
        obj_temp = nightmare_node()
    except rospy.ROSInterruptException:
        pass