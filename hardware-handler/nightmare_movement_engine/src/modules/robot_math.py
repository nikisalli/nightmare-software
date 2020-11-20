import numpy as np
from numpy import sin, cos, tan, arccos, arcsin, arctan, arctan, arctan2, sqrt
from .config import legs, DEFAULT_POSE, SERVO_OFFSET, POSE_OFFSET, POSE_REL_CONVERT

PI = np.pi


def abs_ang2pos(ang):
    angles = np.reshape((ang - SERVO_OFFSET)[:-1], newshape=(6, 3))
    rel_pose = np.array([rel_ang2pos(ang, leg.dim) for ang, leg in zip(angles, legs)])
    pose = (rel_pose * POSE_REL_CONVERT) + POSE_OFFSET
    return pose


def abs_pos2ang(pose):
    rel_poses = (pose - POSE_OFFSET) * POSE_REL_CONVERT
    angles = np.zeros(shape=(6, 3))
    angles = np.array([rel_pos2ang(rel, leg.dim) for rel, leg in zip(rel_poses, legs)])
    return np.append(angles.ravel(), 0) + SERVO_OFFSET


def rel_pos2ang(rel_pos, leg_dim):
    x, y, z = rel_pos
    CX, FM, TB = leg_dim
    d = sqrt(z**2 + (sqrt(x**2 + y**2) - CX)**2)
    alpha = arctan2(x, y)
    beta = arctan((sqrt(x**2 + y**2) - CX) / -z) + arccos((FM**2 + d**2 - TB**2)/(2*FM*d))
    gamma = - arccos((FM**2 + TB**2 - d**2) / (2*FM*TB))
    return np.array([alpha, beta - PI/2, PI - gamma])


def rel_ang2pos(ang, leg_dim):
    alpha, beta, gamma = ang
    CX, FM, TB = leg_dim
    d = CX + cos(beta)*FM + sin(gamma + beta + PI/2) * TB
    x = sin(alpha)*d
    y = cos(alpha)*d
    z = - (cos(gamma + beta + PI/2) * TB - sin(beta) * FM)
    return np.array([x, y, z])
