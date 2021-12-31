import numpy as np
import sys
from dataclasses import dataclass

# ros imports
import rospy

# module imports
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal


class engine_node:
    def __init__(self):
        pinfo("ready")

    def update(self):
        pass


if __name__ == '__main__':
    rospy.init_node('engine')

    pinfo("starting")

    node = engine_node()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    pinfo("stopped")
