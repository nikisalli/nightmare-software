#!/usr/bin/env python3

# ros imports
import rospy
from std_msgs.msg import Byte

class engine_node():
    def __init__(self):
        self.state = 0
        
    def run(self):
        while not rospy.is_shutdown():
            pass
    
    def set_state(self, msg):
        self.state = msg.data


if __name__ == '__main__':
    rospy.init_node('movement_engine')
    
    rospy.loginfo("starting engine node")

    engine = engine_node()
    
    rospy.loginfo("subscribing to state_broadcaster node")
    
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)
    
    engine.run()