import numpy as np
import sys
from dataclasses import dataclass
from numpy import sin, cos, arccos, arctan2, sqrt

# ros imports
import rospy
import tf
from sensor_msgs.msg import JointState

# module imports
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal
from nightmare import config
from nightmare.config import PI, EPSILON


class engine_node:
    def __init__(self):
        # listeners
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/hardware/joint_states", JointState, self.jscb)
        self.js = None

        # hardware data
        self.local_pos = np.zeros(shape=(6, 3))  # hardware position of each leg starting from the attachment to the body
        self.body_pos = np.zeros(shape=(6, 3))  # hardware position of each leg with the center of the body as origin
        self.ground_pos = np.zeros(shape=(6, 3))  # hardware body pos but with rotation relative to the ground

        # wait for first transform to start
        pinfo("waiting for first transform...")
        self.tf_listener.waitForTransform('/body_link', 'leg_1_coxa_1', rospy.Time(0), rospy.Duration(100000000))
        pinfo("ready")

    def compute_ik(self, body_pos):
        def relative_ik(rel_pos, leg_dim):
            x, y, z = rel_pos
            if -EPSILON < z < EPSILON:  # check if z is zero because this could cause division by zero or nan!!!
                z = EPSILON
            CX, FM, TB = leg_dim
            d1 = sqrt(y**2 + x**2) - CX
            d = sqrt(z**2 + (d1)**2)
            alpha = -arctan2(y, x)
            beta = arccos((z**2 + d**2 - d1**2) / (2 * (-z) * d)) + arccos((FM**2 + d**2 - TB**2) / (2 * FM * d))
            gamma = - arccos((FM**2 + TB**2 - d**2) / (2 * FM * TB)) + 2 * PI
            return np.array([alpha, beta - PI / 2, gamma - (PI / 2) * 3])

        rel_poses = (body_pos - config.POSE_OFFSET) * config.POSE_REL_CONVERT
        angles = np.array([relative_ik(rel, leg.dim) for rel, leg in zip(rel_poses, config.legs)])

    def compute_fk(self):
        try:
            for i, leg_tip in enumerate(config.LEG_TIPS):
                self.body_pos[i] = np.array(self.tf_listener.lookupTransform('/body_link', leg_tip, rospy.Time(0))[0])
            # print(self.body_pos)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pwarn("couldn't get transform!")
            return

    def jscb(self, msg):
        self.js = np.array(msg.position)

    def update(self):
        self.compute_fk()
        self.compute_ik(self.body_pos)
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
