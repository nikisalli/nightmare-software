#!/usr/bin/env python3

# standard python imports
import numpy as np
import time

# ros imports
import rospy
from std_msgs.msg import Byte

state = 0

def handle_state():
    rate = rospy.Rate(50)
    pub_state = rospy.Publisher("/nightmare/state", Byte, queue_size=1)
    
    while not rospy.is_shutdown():
        pub_state.publish(state)
        rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('state_broadcaster')
    
    rospy.loginfo("starting state broadcaster")
    handle_state()