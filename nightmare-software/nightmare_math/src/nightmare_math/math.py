import numpy as np
from numpy import sin, cos, arccos, arctan2, sqrt
from nightmare_config.config import legs, SERVO_OFFSET, POSE_OFFSET, POSE_REL_CONVERT
from scipy.spatial.transform import Rotation as R
from nightmare_config.config import (PI,
                                     EPSILON)


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
    gamma = - arccos((FM**2 + TB**2 - d**2) / (2 * FM * TB))
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


def rotation_matrix(pose, rot_vec):
    y_rot = R.from_rotvec(np.array([0, 0, rot_vec[2]]))
    x_rot = R.from_rotvec(np.array([0, rot_vec[0], 0]))
    z_rot = R.from_rotvec(np.array([rot_vec[1], 0, 0]))

    r1 = x_rot.apply(pose)
    r2 = y_rot.apply(r1)
    return z_rot.apply(r2)
