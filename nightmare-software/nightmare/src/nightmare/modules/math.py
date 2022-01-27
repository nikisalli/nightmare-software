import numpy as np
from numpy import sin, cos, arccos, arctan2, sqrt
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from nightmare.config import PI, EPSILON

# ros imports
import rospy
import tf_conversions

# module imports
from nightmare.config import *


def feq(a, b):  # floating point equal
    return abs(a - b) < EPSILON


def no_zero(a):   # make a value always not zero
    if abs(a) < EPSILON:
        return EPSILON
    else:
        return a


def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def asymmetrical_sigmoid(val):
    return 1 / (1 + np.e**(-13 * (val - 0.5)))


def rotate(pose, rot, pivot=None, inverse=False):
    # check if the argument contains an euler rotation vector or a quaternion
    if len(rot) == 3:  # euler
        r = R.from_rotvec(rot)
    elif len(rot) == 4:  # quaternion
        r = R.from_quat(rot)

    if inverse:
        r = r.inv()

    if pivot is not None:  # check if we have a pivot to rotate around
        p = pose - pivot
        p = r.apply(p)
        return p + pivot
    else:
        return r.apply(pose)


def quat2euler(quat):
    return tf_conversions.transformations.euler_from_quaternion(quat)


def euler2quat(euler):
    return tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])


def limit(val, maxv, minv):
    return min(max(val, minv), maxv)
