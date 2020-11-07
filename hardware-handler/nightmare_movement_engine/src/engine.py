#!/usr/bin/env python3

# ros imports
import rospy
from std_msgs.msg import Byte
import tf
from movements import stand_up

class engine_node():
    def __init__(self):
        self.state = 0
        self.prev_state = 0
        
    def run(self):
        while not rospy.is_shutdown():
            if(self.state == 0):
                pass
                #movements.sleep()
    
    def set_state(self, msg):
        self.state = msg.data


if __name__ == '__main__':
    rospy.init_node('movement_engine')
    
    rospy.loginfo("starting engine node")
    engine = engine_node()
    
    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/state", Byte, engine.set_state)
    tf_listener = tf.TransformListener()
    
    engine.run()