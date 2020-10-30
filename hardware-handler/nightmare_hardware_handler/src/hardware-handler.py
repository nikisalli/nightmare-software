#!/usr/bin/env python

import os
import serial
import lewansoul_lx16a
import numpy as np
import math
import time
from beeprint import pp
from math import sin, cos, pi
from threading import Thread

from tf.broadcaster import TransformBroadcaster
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

# robot servo serial ports
tty_list = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]

# simple range mapping function
def fmap(x, in_min, in_max, out_min, out_max):
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min

# servo class to hold servo parameters
class servo:
    tty = ""
    id = 0
    angle = 0
    pos = 0

# leg class to hold servo positions
class leg:
    def __init__(self, num):
        self.num = num
        self.servo = [servo(), servo(), servo()]
            
class listener_thread(Thread): 
    def __init__(self, tty):
        Thread.__init__(self)
        self.tty = tty
    def run(self):
        while not rospy.is_shutdown():
            for leg_num in range(6):
                for servo_num in range(3):
                    id = legs[leg_num].servo[servo_num].id
                    port = legs[leg_num].servo[servo_num].tty
                    if port == self.tty:
                        try:
                            pos = controller[port].get_position(id, timeout=0.02)
                            legs[leg_num].servo[servo_num].pos = pos
                            legs[leg_num].servo[servo_num].angle = round(fmap(pos, -500, 1500, -3.14, 3.14),4)
                        except:
                            rospy.logerr("couldn't read servo position. ID: " + str(id) + " port: ttyUSB" + str(port))
            time.sleep(0.05) # execute every 0.05s

# main node class
class nightmare_node():
    def __init__(self):
        self.fixed_frame = rospy.get_param('~fixed_frame', "world") # set fixed frame relative to world to apply the transform
        self.frame = rospy.get_param('~frame', "base_link") #set frame name
        self.pub_jnt = rospy.Publisher("joint_states", JointState, queue_size=10) # joint state publisher
        self.odom_broadcaster = TransformBroadcaster()
        
        self.jnt_msg = JointState() # joint topic structure
        
        self.jnt_msg.header = Header()
        self.jnt_msg.name = ['Rev103', 'Rev104', 'Rev105', 'Rev106', 'Rev107', 'Rev108',
                             'Rev109', 'Rev110', 'Rev111', 'Rev112', 'Rev113', 'Rev114',
                             'Rev115', 'Rev116', 'Rev117', 'Rev118', 'Rev119', 'Rev120',
                             'Rev121']
        
        """self.jnt_msg.name = ['Rev105', 'Rev111', 'Rev117', 'Rev104', 'Rev110', 'Rev116',
                             'Rev103', 'Rev109', 'Rev115', 'Rev108', 'Rev114', 'Rev120',
                             'Rev107', 'Rev113', 'Rev119', 'Rev106', 'Rev112', 'Rev118', 
                             'Rev121']"""
        self.jnt_msg.velocity = []
        self.jnt_msg.effort = []
        
        rospy.on_shutdown(self.shutdown_node)
        
        rospy.loginfo("Ready for publishing")
        
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            self.odom_broadcaster.sendTransform((0,0,0), 
                                                (0,0,0,0), 
                                                rospy.Time.now(), 
                                                self.fixed_frame, 
                                                self.frame)
            
            self.publish_jnt()
            
            rate.sleep()
            
    def publish_jnt(self):
        angles = [0]*18
        
        for leg_num in range(6):
            for servo_num in range(3):
                angles[(leg_num * 3) + servo_num] = legs[leg_num].servo[servo_num].angle
        
        #print(angles)
        
        self.jnt_msg.position = angles
        
        self.jnt_msg.position = [ angles[2],  angles[8],  angles[14], angles[1],  angles[7],  angles[13],
                                  angles[0],  angles[6],  angles[12], angles[5],  angles[11], angles[17],
                                  angles[4],  angles[10], angles[16], angles[3],  angles[9],  angles[15], 0]

        self.jnt_msg.header.stamp = rospy.Time.now()
        self.pub_jnt.publish(self.jnt_msg)

    def shutdown_node(self):
        rospy.loginfo("shutting down hardware handler node")
        
if __name__ == '__main__':
    rospy.loginfo("starting hardware handler node")
    
    rospy.init_node('hardware_handler')
    
    #-------------------------------------------
    
    rospy.loginfo("setting ports to low latency mode")
    
    for tty in tty_list:
        os.system('setserial ' + tty + ' low_latency')
    
    #-------------------------------------------
    
    rospy.loginfo("creating controller objects")
    
    controller = [0]*4
    for i, tty in enumerate(tty_list):
        controller[i] = lewansoul_lx16a.ServoController(serial.Serial(tty, 115200, timeout=1))
    
    #-------------------------------------------

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
    
    #-------------------------------------------
    
    rospy.loginfo("spawning reading threads")
    
    thread_list = []
    
    for i in range(len(tty_list)):
        thread_list.append(listener_thread(i))
        thread_list[i].start()
        
    try:
        node = nightmare_node()
    except rospy.ROSInterruptException:
        pass