import numpy as np
from numpy import sin
import time
from .robot_math import abs_pos2ang
from .config import DEFAULT_POSE


def sleep(engine):
    engine.pose = DEFAULT_POSE.copy()
    engine.pose[:, 2] += sin(time.time()*2)*10
    engine.angles_array = abs_pos2ang(engine.pose)
