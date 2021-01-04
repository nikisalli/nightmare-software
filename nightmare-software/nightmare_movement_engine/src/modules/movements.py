from random import randint
import time
import numpy as np
from nightmare_math.math import (asymmetrical_sigmoid,
                                 rotation_matrix)

from nightmare_config.config import (DEFAULT_POSE,
                                     STEP_HEIGHT,
                                     ENGINE_FPS,
                                     TIME_STAND_UP,
                                     DEFAULT_SIT_POSE,
                                     DOUBLE_SEQUENCES,
                                     STAND_HEIGTH,
                                     SIT_HEIGHT,
                                     TIME_SIT)

import bezier


def stand_up(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_SIT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
        curve = bezier.Curve(nodes, degree=3)
        curves.append(curve)
    seq_num = randint(0, 1)
    engine.pose = engine.hw_pose.copy()
    for pair in DOUBLE_SEQUENCES[seq_num]:
        for i in range(int((TIME_STAND_UP * ENGINE_FPS) / 3)):
            engine.pose[pair[0] - 1] = np.array(curves[pair[0] - 1].evaluate(i / ((TIME_STAND_UP * ENGINE_FPS) / 3))).flatten()
            engine.pose[pair[1] - 1] = np.array(curves[pair[1] - 1].evaluate(i / ((TIME_STAND_UP * ENGINE_FPS) / 3))).flatten()
            engine.compute_ik()
            time.sleep(TIME_STAND_UP / (3 * ENGINE_FPS))
    for i in range(int(TIME_STAND_UP * ENGINE_FPS)):
        engine.pose = DEFAULT_SIT_POSE.copy()
        engine.pose[:, 2] += asymmetrical_sigmoid(i / (TIME_STAND_UP * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.compute_ik()
        time.sleep(0.01)


def sit(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
        curve = bezier.Curve(nodes, degree=3)
        curves.append(curve)
    seq_num = randint(0, 1)
    engine.pose = engine.hw_pose.copy()
    for pair in DOUBLE_SEQUENCES[seq_num]:
        for i in range(int((TIME_SIT * ENGINE_FPS) / 3)):
            engine.pose[pair[0] - 1] = np.array(curves[pair[0] - 1].evaluate(i / ((TIME_SIT * ENGINE_FPS) / 3))).flatten()
            engine.pose[pair[1] - 1] = np.array(curves[pair[1] - 1].evaluate(i / ((TIME_SIT * ENGINE_FPS) / 3))).flatten()
            engine.compute_ik()
            time.sleep(TIME_SIT / (3 * ENGINE_FPS))
    for i in range(int(TIME_SIT * ENGINE_FPS)):
        engine.pose = DEFAULT_POSE.copy()
        engine.pose[:, 2] -= asymmetrical_sigmoid(i / (TIME_SIT * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.compute_ik()
        time.sleep(0.01)


def stand(engine, disp):
    pose = rotation_matrix(DEFAULT_POSE.copy(), disp[-3:])
    pose[:, 0] += disp[1]
    pose[:, 1] += disp[0]
    pose[:, 2] += disp[2]
    engine.pose = pose
    engine.compute_ik()


def sleep(engine):
    engine.pose = DEFAULT_SIT_POSE.copy()
    engine.compute_ik()
