#!/usr/bin/env python3
# pylint: disable=broad-except, no-name-in-module

from random import randint
import time
import numpy as np

import rospy

from nightmare_math.math import (asymmetrical_sigmoid,
                                 rotate,
                                 abs2relpos,
                                 rel2abspos,
                                 quat2euler,
                                 limit)

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
                                     GAIT,
                                     EPSILON,
                                     ENGINE_REFERENCE_FRAME,
                                     DYNAMIC_TERRAIN_ADAPTATION_ENABLED)

import bezier


def execute_step(engine):
    step = engine.steps.pop(0)  # remove it from the step stack

    engine.step_id = step['id']  # save its id so we know what we did
    # rospy.loginfo(f"executing id {engine.step_id}, steps in queue {len(engine.steps)}")
    engine.publish_step_id()

    trans = engine.tf_buffer.lookup_transform(ENGINE_REFERENCE_FRAME, 'base_link', rospy.Time(0), rospy.Duration(.1)).transform
    start_trasl = [trans.translation.x, trans.translation.y, trans.translation.z]
    start_rot = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
    rospy.loginfo(str(quat2euler(start_rot)))
    # start_trasl = engine.body_abs_trasl.copy()
    # start_rot = engine.body_abs_rot.copy()
    abs_pose = rel2abspos(engine.hw_pose, start_trasl, start_rot)  # put this in main engine class and handle it TODO
    curves = {}

    for substep in step['steps']:
        leg = substep['leg']
        start_pose = abs_pose[leg]
        end_pose = np.array(substep['pos'])

        nodes = [[start_pose[0], start_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2], start_pose[2] + STEP_HEIGHT, end_pose[2] + STEP_HEIGHT, end_pose[2]]]
        curves[leg] = bezier.Curve(nodes, degree=3)

    divider = len(GAIT[engine.gait])
    frames = int(STEP_TIME * 0.5 * ENGINE_FPS)

    for i in range(frames):
        timer = time.time()

        trasl_command = (engine.walk_trasl * engine.attenuation * 2) / (frames * divider)
        rot_command = (engine.walk_rot * engine.attenuation * 2) / (frames * divider)
        rel_rotated_command = rotate(trasl_command, rot_command)
        abs_rotated_command = rotate(trasl_command, rot_command + engine.body_abs_rot)

        for leg in range(NUMBER_OF_LEGS):
            if leg in curves:
                pose = np.array(curves[leg].evaluate(i / frames)).flatten()
                pose = abs2relpos(pose, start_trasl, start_rot)
                engine.pose[leg] = pose
            else:
                # level legs only if on ground
                # leg_correction(engine, leg)
                pose = engine.pose[leg].copy()
                pose = rotate(pose, rot_command, inverse=True)
                pose -= rel_rotated_command
                engine.pose[leg] = pose

        # generate odom
        engine.body_abs_rot += rot_command
        engine.body_abs_trasl += abs_rotated_command

        apply_transform(engine)

        engine.compute_ik()
        if (1 / ENGINE_FPS) - (time.time() - timer) > EPSILON:
            time.sleep((1 / ENGINE_FPS) - (time.time() - timer))


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
        pos = asymmetrical_sigmoid(i / (TIME_STAND_UP * ENGINE_FPS)) * (- STAND_HEIGTH - SIT_HEIGHT)
        engine.final_pose[:, 2] += pos

        engine.final_body_abs_trasl[2] = - SIT_HEIGHT - pos  # inverted because when legs go down, robot goes up

        engine.compute_ik()
        time.sleep(0.05)


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
        pos = asymmetrical_sigmoid(i / (TIME_SIT * ENGINE_FPS)) * (- STAND_HEIGTH - SIT_HEIGHT)
        engine.final_pose[:, 2] -= pos

        engine.final_body_abs_trasl[2] = STAND_HEIGTH + pos  # inverted because when legs go down, robot goes up

        engine.compute_ik()
        time.sleep(0.05)


def stand(engine):
    # level legs
    if(DYNAMIC_TERRAIN_ADAPTATION_ENABLED):
        leg_correction(engine)
    engine.body_abs_trasl[2] = STAND_HEIGTH
    apply_transform(engine)
    engine.compute_ik()


def apply_transform(engine):
    # apply transform
    pose_correction(engine)
    pose = rotate(engine.pose.copy(), engine.body_rot, inverse=True)
    pose = rotate(pose, [engine.roll_correction, engine.pitch_correction, 0])
    pose -= engine.body_trasl
    if(DYNAMIC_TERRAIN_ADAPTATION_ENABLED):
        pose[:, 2] += engine.leg_z_correction
    engine.final_pose = pose

    # generate odom
    engine.final_body_abs_rot = engine.body_abs_rot + engine.body_rot
    rotated_command = rotate(engine.body_trasl, engine.body_rot + engine.body_abs_rot)
    engine.final_body_abs_trasl = engine.body_abs_trasl + rotated_command


def pose_correction(engine):
    trans = engine.tf_buffer.lookup_transform(ENGINE_REFERENCE_FRAME, 'base_link', rospy.Time(0), rospy.Duration(.1)).transform
    rot = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
    rot_euler = quat2euler(rot)
    engine.roll_correction += engine.pose_filter_val * (rot_euler[0] - engine.pose_roll_setpoint)
    engine.pitch_correction += engine.pose_filter_val * (rot_euler[1] - engine.pose_pitch_setpoint)


def leg_correction(engine):
    for leg in range(NUMBER_OF_LEGS):
        engine.leg_z_correction[leg] -= engine.leg_adapt_filter_val * (engine.leg_adapt_var_filtered_setpoint - engine.force_sensors[leg])
        engine.leg_z_correction[leg] = limit(engine.leg_z_correction[leg], engine.max_leg_adapt_adjust, - engine.max_leg_adapt_adjust)  # to make sure it doesn't grow without control
    avg = float(sum(engine.leg_z_correction) / NUMBER_OF_LEGS)
    engine.leg_z_correction -= avg


def sleep(engine):
    engine.final_pose = DEFAULT_SIT_POSE.copy()
    engine.body_abs_trasl[2] = - SIT_HEIGHT

    engine.compute_ik()
