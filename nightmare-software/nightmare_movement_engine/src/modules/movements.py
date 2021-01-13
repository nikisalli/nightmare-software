from random import randint
import time
import numpy as np
from numpy import sin, cos

import rospy

import tf_conversions

from nightmare_math.math import (asymmetrical_sigmoid,
                                 euler_rotation_matrix)

from nightmare_config.config import (DEFAULT_POSE,
                                     STEP_HEIGHT,
                                     STEP_TIME,
                                     ENGINE_FPS,
                                     TIME_STAND_UP,
                                     DEFAULT_SIT_POSE,
                                     DOUBLE_SEQUENCES,
                                     STAND_HEIGTH,
                                     SIT_HEIGHT,
                                     TIME_SIT,
                                     NUMBER_OF_LEGS,
                                     PI)

import bezier


def step(engine):
    stp = engine.steps.pop(0)  # remove it from the step stack
    step_id = stp['id']  # save its id so we know what we did
    rospy.loginfo(f"executing id {step_id}, steps in queue {len(engine.steps)}")
    engine.step_id = step_id
    engine.publish_step_id()
    curves = {}
    trsfs = {}

    # find legs involved in the new step by iterating over the step array
    for substp in stp['steps']:
        trsfs[substp['leg']] = substp['pos']

    dpose = euler_rotation_matrix(DEFAULT_POSE, [0, 0, PI / 2])

    # generate curve going from prev pos to new pos using bezier
    for leg, start_pose in enumerate(engine.hw_pose.copy()):
        if leg in trsfs:
            end_pose = dpose[leg] + trsfs[leg]
            end_pose = euler_rotation_matrix(end_pose, [0, 0, -PI / 2])
            nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                     [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                     [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
            curve = bezier.Curve(nodes, degree=3)
            curves[leg] = curve

    starttime = time.time()

    divider = 0

    if engine.gait == 'tripod':
        divider = 2

    duration = 1 / (ENGINE_FPS * STEP_TIME * divider)

    for i in range(int(STEP_TIME * ENGINE_FPS)):
        for leg in range(NUMBER_OF_LEGS):
            if leg in curves:
                engine.pose[leg] = np.array(curves[leg].evaluate(i / (STEP_TIME * ENGINE_FPS))).flatten()
            else:
                new_pose = engine.pose[leg].copy() + ((np.asarray([engine.walk_direction[1] * engine.attenuation, - engine.walk_direction[0] * engine.attenuation, 0.0]) * 2) / - (STEP_TIME * ENGINE_FPS))
                new_pose = euler_rotation_matrix(new_pose, [0, 0, (engine.walk_direction[2] * engine.attenuation / (STEP_TIME * ENGINE_FPS)) * 2])
                engine.pose[leg] = new_pose

        # generate odom
        engine.body_rot[2] += 2 * (- engine.walk_direction[2] * engine.attenuation / (STEP_TIME * ENGINE_FPS))
        engine.body_trasl[0] += 2 * (- (engine.walk_direction[1] * engine.attenuation / (STEP_TIME * ENGINE_FPS)) * sin(engine.body_rot[2]) + (engine.walk_direction[0] * engine.attenuation / (STEP_TIME * ENGINE_FPS)) * cos(engine.body_rot[2]))
        engine.body_trasl[1] += 2 * ((engine.walk_direction[1] * engine.attenuation / (STEP_TIME * ENGINE_FPS)) * cos(engine.body_rot[2]) + (engine.walk_direction[0] * engine.attenuation / (STEP_TIME * ENGINE_FPS)) * sin(engine.body_rot[2]))

        apply_transform(engine)

        engine.compute_ik()
        time.sleep((i + 1) * duration - time.time() + starttime)


def stand_up(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_SIT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
        curve = bezier.Curve(nodes, degree=3)
        curves.append(curve)
    seq_num = randint(0, 1)
    engine.final_pose = engine.hw_pose.copy()
    for pair in DOUBLE_SEQUENCES[seq_num]:
        for i in range(int((TIME_STAND_UP * ENGINE_FPS) / 3)):
            leg1 = pair[0] - 1  # subtract 1 because legs go from 1 to 6 but arrays start at 0
            leg2 = pair[1] - 1
            engine.final_pose[leg1] = np.array(curves[leg1].evaluate(i / ((TIME_STAND_UP * ENGINE_FPS) / 3))).flatten()
            engine.final_pose[leg2] = np.array(curves[leg2].evaluate(i / ((TIME_STAND_UP * ENGINE_FPS) / 3))).flatten()
            engine.compute_ik()
            time.sleep(TIME_STAND_UP / (3 * ENGINE_FPS))
    for i in range(int(TIME_STAND_UP * ENGINE_FPS)):
        engine.final_pose = DEFAULT_SIT_POSE.copy()
        pos = asymmetrical_sigmoid(i / (TIME_STAND_UP * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.final_pose[:, 2] += pos

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
    engine.final_pose = engine.hw_pose.copy()
    for pair in DOUBLE_SEQUENCES[seq_num]:
        for i in range(int((TIME_SIT * ENGINE_FPS) / 3)):
            engine.final_pose[pair[0] - 1] = np.array(curves[pair[0] - 1].evaluate(i / ((TIME_SIT * ENGINE_FPS) / 3))).flatten()
            engine.final_pose[pair[1] - 1] = np.array(curves[pair[1] - 1].evaluate(i / ((TIME_SIT * ENGINE_FPS) / 3))).flatten()
            engine.compute_ik()
            time.sleep(TIME_SIT / (3 * ENGINE_FPS))
    for i in range(int(TIME_SIT * ENGINE_FPS)):
        engine.final_pose = DEFAULT_POSE.copy()
        pos = asymmetrical_sigmoid(i / (TIME_SIT * ENGINE_FPS)) * (STAND_HEIGTH - SIT_HEIGHT)
        engine.final_pose[:, 2] -= pos

        engine.engine_pos_publisher_msg.transform.translation.z = - STAND_HEIGTH + pos  # inverted because when legs go down, robot goes up

        engine.compute_ik()
        time.sleep(0.01)


def stand(engine):
    apply_transform(engine)
    engine.compute_ik()


def apply_transform(engine):
    # apply transform
    pose = euler_rotation_matrix(engine.pose.copy(), engine.body_displacement[-3:])
    pose[:, 0] += engine.body_displacement[1]
    pose[:, 1] += engine.body_displacement[0]
    pose[:, 2] += engine.body_displacement[2]
    engine.final_pose = pose

    # generate odom
    engine.engine_pos_publisher_msg.transform.translation.x = engine.body_displacement[0] + engine.body_trasl[0]
    engine.engine_pos_publisher_msg.transform.translation.y = - engine.body_displacement[1] + engine.body_trasl[1]
    engine.engine_pos_publisher_msg.transform.translation.z = - STAND_HEIGTH - engine.body_displacement[2]
    # convert rpy to ros friendly quaternion
    q = tf_conversions.transformations.quaternion_from_euler(engine.body_displacement[3], - engine.body_displacement[4], engine.body_rot[2])
    engine.engine_pos_publisher_msg.transform.rotation.x = q[0]
    engine.engine_pos_publisher_msg.transform.rotation.y = q[1]
    engine.engine_pos_publisher_msg.transform.rotation.z = q[2]
    engine.engine_pos_publisher_msg.transform.rotation.w = q[3]


def sleep(engine):
    engine.final_pose = DEFAULT_SIT_POSE.copy()
    engine.engine_pos_publisher_msg.transform.translation.z = - SIT_HEIGHT

    # generate odom
    engine.engine_pos_publisher_msg.transform.translation.x = engine.body_trasl[0]
    engine.engine_pos_publisher_msg.transform.translation.y = engine.body_trasl[1]
    engine.engine_pos_publisher_msg.transform.translation.z = engine.body_trasl[2]
    # convert rpy to ros friendly quaternion
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, engine.body_rot[2])
    engine.engine_pos_publisher_msg.transform.rotation.x = q[0]
    engine.engine_pos_publisher_msg.transform.rotation.y = q[1]
    engine.engine_pos_publisher_msg.transform.rotation.z = q[2]
    engine.engine_pos_publisher_msg.transform.rotation.w = q[3]

    engine.compute_ik()
