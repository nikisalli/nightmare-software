import numpy as np
from numpy import sin, cos, tan, arccos, arcsin, arctan, arctan, arctan2, sqrt

PI = np.pi


def abs_pos2ang():
    pass


def rel_pos2ang(rel_pos, leg_dim):
    x = rel_pos[0]
    y = rel_pos[1]
    z = rel_pos[2]
    CX = leg_dim[0]
    FM = leg_dim[1]
    TB = leg_dim[2]
    d = sqrt(x**2 + y**2)
    d1 = d - CX
    d2 = sqrt(z**2 + d1**2)
    alpha = arctan2(y, x)
    beta = arctan(d1 / -z) + arccos((FM**2 + d2**2 - TB**2)/(2*FM*d2))
    gamma = - arccos((FM**2 + TB**2 - d2**2) / (2*FM*TB))
    return np.array([alpha, beta - PI/2, PI - gamma])


def rel_ang2pos(ang, leg_dim):
    alpha = ang[0]
    beta = ang[1]
    gamma = ang[2]
    CX = leg_dim[0]
    FM = leg_dim[1]
    TB = leg_dim[2]
    d = CX + cos(beta)*FM + sin(gamma - PI/2 + beta)
    x = sin(alpha)*d
    y = cos(alpha)*d
    z = cos(gamma - PI/2 + beta) + TB - sin(beta) * FM
    return np.array([x, y, z])
