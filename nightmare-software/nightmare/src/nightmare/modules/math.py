import numpy as np
from numpy import sin, cos, arccos, arctan2, sqrt
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

# ros imports
import rospy
import tf_conversions

# module imports
from nightmare.config import *


RIGHT = 1
LEFT = 0
PI = np.pi
EPSILON = 0.0001


def feq(a, b):  # floating point equal
    return abs(a - b) < EPSILON


def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def abs_ang2pos(ang):
    angles = np.reshape((ang - SERVO_OFFSET)[:-1], newshape=(6, 3))
    rel_pose = np.array([rel_ang2pos(ang, leg.dim) for ang, leg in zip(angles, legs)])
    return (rel_pose * POSE_REL_CONVERT) + POSE_OFFSET


def abs_pos2ang(pose):
    rel_poses = (pose - POSE_OFFSET) * POSE_REL_CONVERT
    angles = np.zeros(shape=(6, 3))
    angles = np.array([rel_pos2ang(rel, leg.dim) for rel, leg in zip(rel_poses, legs)])
    return np.append(angles.ravel(), 0) + SERVO_OFFSET


def rel_pos2ang(rel_pos, leg_dim):
    x, y, z = rel_pos
    if -EPSILON < z < EPSILON:  # check if z is zero because this could cause division by zero or nan!!!
        z = EPSILON
    CX, FM, TB = leg_dim
    d1 = sqrt(x**2 + y**2) - CX
    d = sqrt(z**2 + (d1)**2)
    alpha = arctan2(x, y)
    beta = arccos((z**2 + d**2 - d1**2) / (2 * (-z) * d)) + arccos((FM**2 + d**2 - TB**2) / (2 * FM * d))
    gamma = - arccos((FM**2 + TB**2 - d**2) / (2 * FM * TB)) + 2 * PI
    return np.array([alpha, beta - PI / 2, PI - gamma])


def rel_ang2pos(ang, leg_dim):
    alpha, beta, gamma = ang
    CX, FM, TB = leg_dim
    d = CX + cos(beta) * FM + sin(gamma + beta + PI / 2) * TB
    x = sin(alpha) * d
    y = cos(alpha) * d
    z = - (cos(gamma + beta + PI / 2) * TB - sin(beta) * FM)
    return np.array([x, y, z])


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


def abs2relpos(abs_pose, trasl, rot):
    rel_pose = abs_pose
    rel_pose -= trasl
    rel_pose = rotate(rel_pose, rot, inverse=True)
    return rel_pose


def rel2abspos(rel_pose, trasl, rot):
    abs_pose = rel_pose
    abs_pose = rotate(rel_pose, rot)
    abs_pose += trasl
    return abs_pose


def quat2euler(quat):
    return tf_conversions.transformations.euler_from_quaternion(quat)


def euler2quat(euler):
    return tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])


def limit(val, maxv, minv):
    return min(max(val, minv), maxv)
