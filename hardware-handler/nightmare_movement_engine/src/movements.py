import numpy as np
from numpy import sin
import time
from robot_math.inverse_kinematics import rel_pos2ang
from robot_config.config import legs, DEFAULT_POSE, SERVO_OFFSET, POSE_OFFSET


def sleep(engine):
    pass


def test(engine):
    engine.pose[5][2] = -16 + sin(time.time())*10
    print(engine.pose)
    engine.angles[5] = rel_pos2ang(np.subtract(engine.pose[5], POSE_OFFSET[5]), legs[5].dim)
    engine.angles_array = np.add(np.append(engine.angles.ravel(), 0), SERVO_OFFSET)
    # print(engine.angles_array)
