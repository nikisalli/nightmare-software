from random import randint
import time
import numpy as np

import tf_conversions

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


def step(engine):
    pass


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
        pos = asymmetrical_sigmoid(i / (TIME_STAND_UP * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.pose[:, 2] += pos
        engine.engine_pos_publisher_msg.transform.translation.z = - SIT_HEIGHT - pos  # inverted because when legs go down, robot goes up
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
        pos = asymmetrical_sigmoid(i / (TIME_SIT * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.pose[:, 2] -= pos
        engine.engine_pos_publisher_msg.transform.translation.z = - STAND_HEIGTH + pos  # inverted because when legs go down, robot goes up
        engine.compute_ik()
        time.sleep(0.01)


def stand(engine):
    pose = rotation_matrix(DEFAULT_POSE.copy(), engine.body_displacement[-3:])
    pose[:, 0] += engine.body_displacement[1]
    pose[:, 1] += engine.body_displacement[0]
    pose[:, 2] += engine.body_displacement[2]
    engine.pose = pose
    engine.engine_pos_publisher_msg.transform.translation.x = engine.body_displacement[0]
    engine.engine_pos_publisher_msg.transform.translation.y = - engine.body_displacement[1]
    engine.engine_pos_publisher_msg.transform.translation.z = - STAND_HEIGTH - engine.body_displacement[2]
    q = tf_conversions.transformations.quaternion_from_euler(engine.body_displacement[3], - engine.body_displacement[4], 0)
    engine.engine_pos_publisher_msg.transform.rotation.x = q[0]
    engine.engine_pos_publisher_msg.transform.rotation.y = q[1]
    engine.engine_pos_publisher_msg.transform.rotation.z = q[2]
    engine.engine_pos_publisher_msg.transform.rotation.w = q[3]
    engine.compute_ik()


def sleep(engine):
    engine.pose = DEFAULT_SIT_POSE.copy()
    engine.engine_pos_publisher_msg.transform.translation.z = - SIT_HEIGHT
    engine.compute_ik()
