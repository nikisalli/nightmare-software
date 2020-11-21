import numpy as np
from numpy import sin
import time
from .config import DEFAULT_POSE, STEP_HEIGHT, ENGINE_FPS, TIME_STAND_UP
import bezier


def stand_up(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
        curve = bezier.Curve(nodes, degree=3)
        curves.append(curve)
    for i in range(int(TIME_STAND_UP * ENGINE_FPS)):
        for j, curve in enumerate(curves):
            engine.pose[j] = np.array(curve.evaluate(i / int(TIME_STAND_UP * ENGINE_FPS))).flatten()

        engine.compute_ik()
        time.sleep(1 / ENGINE_FPS)


def sleep(engine):
    engine.pose = DEFAULT_POSE.copy()
    engine.pose[:, 2] += sin(time.time()*2)*10
    engine.compute_ik()
