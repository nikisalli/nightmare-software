import numpy as np
import sys
from dataclasses import dataclass

# ros imports
import rospy

# module imports
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal
from nightmare.config import *


class engine_node:
    def __init__(self):
        # hardware data
        self.local_pos = np.zeros(shape=(3, 6))  # hardware position of each leg starting from the attachment to the body
        self.body_pos = np.zeros(shape=(3, 6))  # hardware position of each leg with the center of the body as origin
        self.ground_pos = np.zeros(shape=(3, 6))  # hardware body pos but with rotation relative to the ground
        pinfo("ready")

    def compute_ik(self, ground_pos):
        pass

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
