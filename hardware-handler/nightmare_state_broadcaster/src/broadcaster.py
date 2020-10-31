#!/usr/bin/env python

import rospy
import numpy as np
import time
from std_msgs.msg import Byte

state = 0

def handle_state():
    rate = rospy.Rate(50)
    pub_state = rospy.Publisher("nightmare/state", Byte, queue_size=1)
    
    while True:
        pub_state.publish(state)
        rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('state_broadcaster')
    
    rospy.loginfo("starting state broadcaster")
    handle_state()