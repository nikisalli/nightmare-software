#!/usr/bin/env python

import rospy
import serial
import lewansoul_lx16a
import numpy as np
import tf
import math
import time
from beeprint import pp

from threading import Thread
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster

tty_list = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]                        #robot serial port

def fmap(x, in_min, in_max, out_min, out_max):
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min

class servo:
    tty = ""
    id = 0
    angle = 0
    pos = 0

class leg:
    def __init__(self, num):
        self.num = num
        self.servo = [servo(), servo(), servo()]
        
class robot_listener(Thread):                                                                      #robot listener to parse data coming from the robot
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while True:
            t = time.time()
            for leg_num in range(6):
                for servo_num in range(3):
                    try:
                        id = legs[leg_num].servo[servo_num].id
                        port = legs[leg_num].servo[servo_num].tty
                        pos = controller[port].get_position(id, timeout=0.02)
                        legs[leg_num].servo[servo_num].pos = pos
                        legs[leg_num].servo[servo_num].angle = round(fmap(pos, 0, 1000, -120, 120),2)
                    except Exception as e:
                        print(e)
                        pass
            print(time.time()-t)

class nightmare_node():
    def __init__(self):
        self.rate = rospy.get_param('~rate', 30.0)                                                #the rate at which to publish the transform

        self.fixed_frame = rospy.get_param('~fixed_frame', "world")                                #set fixed frame to apply the transform
        self.publish_transform = rospy.get_param('~publish_transform', False)                      #set true in lunch file if the transform is needed

        self.pub_jnt = rospy.Publisher("nightmare/joint_states", JointState, queue_size=10)

        self.jnt_msg = JointState()
        self.jnt_msg.header = Header()
        self.jnt_msg.name = ['Rev24', 'Rev25', 'Rev26', 'Rev27', 'Rev28', 'Rev29',
                             'Rev30', 'Rev31', 'Rev32', 'Rev33', 'Rev34', 'Rev35',
                             'Rev36', 'Rev37', 'Rev38', 'Rev39', 'Rev48', 'Rev49', 
                             'Rev50']
        self.jnt_msg.velocity = []
        self.jnt_msg.effort = []

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(30) # 100hz

        rospy.loginfo("Ready for publishing")

        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()
            self.publish_jnt()

            rate.sleep()

    def publish_jnt(self):
        angles = [0]*18
        
        for leg_num in range(6):
            for servo_num in range(3):
                angles[(leg_num * 3) + servo_num] = legs[leg_num].servo[servo_num].angle
        
        print(angles)
        
        self.jnt_msg.position = angles
        
        """self.jnt_msg.position = [-angles[0], -angles[1], -angles[2], -angles[3], -angles[4], -angles[5],
                                  angles[6],  angles[7],  angles[8],  angles[9],  angles[10], angles[11],
                                  angles[12], angles[13], angles[14], angles[15], angles[16], angles[17], 0]"""

        self.jnt_msg.header.stamp = rospy.Time.now()
        self.pub_jnt.publish(self.jnt_msg)

    def shutdown_node(self):
        rospy.loginfo("shutting down hardware handler node")

if __name__ == '__main__':
    rospy.loginfo("starting hardware handler node")
    
    rospy.init_node('hardware_handler')
    
    rospy.loginfo("creating controller objects")
    
    controller = [0]*4
    for i, tty in enumerate(tty_list):
        controller[i] = lewansoul_lx16a.ServoController(serial.Serial(tty, 115200, timeout=1))
    
    rospy.loginfo("indexing servos")
    
    legs = []
    for i in range(6):
        legs.append(leg(i))
        
    for leg_num in range(6):
        for servo_num in range(3):
            for tty_num, tty in enumerate(tty_list):
                id = (leg_num * 3) + servo_num + 1
                try:
                    controller[tty_num].get_servo_id(id, timeout=0.05)
                    rospy.loginfo("found servo id " + str(id) + " on " + str(tty))
                    legs[leg_num].servo[servo_num].id = id
                    legs[leg_num].servo[servo_num].tty = tty_num
                    break
                except:
                    pass
        
    robot_listener()
    
    try:
        pass
        obj_temp = nightmare_node()
    except rospy.ROSInterruptException:
        pass