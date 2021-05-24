#!/usr/bin/env python3
# pylint: disable=broad-except, no-name-in-module

from random import randint
import time
import numpy as np

import rospy
from random import randint

from nightmare_math.math import (asymmetrical_sigmoid,
                                 rotate,
                                 abs2relpos,
                                 rel2abspos,
                                 quat2euler,
                                 limit, fmap)

from nightmare_config.config import (DEFAULT_POSE,
                                     STEP_HEIGHT,
                                     STEP_TIME,
                                     ENGINE_FPS,
                                     TIME_STAND_UP,
                                     DEFAULT_SIT_POSE,
                                     DOUBLE_SEQUENCES,
                                     STAND_HEIGHT,
                                     SIT_HEIGHT,
                                     TIME_SIT,
                                     NUMBER_OF_LEGS,
                                     GAIT,
                                     EPSILON,
                                     ENGINE_REFERENCE_FRAME,
                                     DYNAMIC_TERRAIN_ADAPTATION_ENABLED,
                                     TOUCHDOWN_TRESHOLD,
                                     MAX_TOUCHDOWN_DEPTH_SEARCH,
                                     WALKING_TERRAIN_ADAPTATION_ENABLED,
                                     AUTO_HEIGHT_ADJUSTMENT_ENABLED,
                                     SPRINGINESS,
                                     DAMPINESS,
                                     LEG_ADJ_HEIGHT)

import bezier


def execute_step(engine):
    step = engine.steps.pop(0)  # remove it from the step queue

    engine.step_id = step['id']  # save its id so we know what we did
    # rospy.loginfo(f"executing id {engine.step_id}, steps in queue {len(engine.steps)}")
    engine.publish_step_id()

    trans = engine.tf_buffer.lookup_transform(ENGINE_REFERENCE_FRAME, 'base_link', rospy.Time(0), rospy.Duration(.1)).transform
    start_trasl = [trans.translation.x, trans.translation.y, trans.translation.z]
    start_rot = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
    # rospy.loginfo(str(start_trasl))
    # start_trasl = engine.body_abs_trasl.copy()
    # start_rot = engine.body_abs_rot.copy()
    abs_pose = rel2abspos(engine.pose, start_trasl, start_rot)  # put this in main engine class and handle it TODO
    curves = {}

    for substep in step['steps']:
        leg = substep['leg']
        start_pose = abs_pose[leg]
        end_pose = np.array(substep['pos'])

        nodes = [[start_pose[0], start_pose[0], start_pose[0], end_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], start_pose[1], end_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2],
                  start_pose[2] + (STEP_HEIGHT / 6),
                  start_pose[2] + STEP_HEIGHT,
                  end_pose[2] + STEP_HEIGHT,
                  end_pose[2] + (STEP_HEIGHT / 6),
                  end_pose[2]]]
        print(nodes)
        curves[leg] = bezier.Curve(nodes, degree=5)

    divider = len(GAIT[engine.gait])
    frames = int(STEP_TIME * 0.5 * ENGINE_FPS)

    # array to hold leg state and handle debounce
    # The not suspended legs are set to True by default because we know those are on ground
    engine.touched = np.array([leg not in curves for leg in range(NUMBER_OF_LEGS)])  # length NUMBER_OF_LEGS

    for i in range(frames + 1):
        timer = time.time()

        trasl_command = (engine.walk_trasl * engine.attenuation * 2) / (frames * divider)
        rot_command = (engine.walk_rot * engine.attenuation * 2) / (frames * divider)
        rel_rotated_command = rotate(trasl_command, rot_command)
        abs_rotated_command = rotate(trasl_command, rot_command + engine.body_abs_rot)

        for leg in range(NUMBER_OF_LEGS):
            if leg in curves:
                if True:
                    # if not engine.touched[leg]:
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

        print(engine.pose[:, 2], i / frames)
        # generate odom
        engine.body_abs_rot += rot_command
        engine.body_abs_trasl += abs_rotated_command

        apply_transform(engine)

        engine.compute_ik()
        # if a leg is false and it touched ground set its corresponding value to true
        # start checking only when the leg left the ground!
        if i > frames / 2:
            for leg in range(NUMBER_OF_LEGS):
                if engine.leg_ground[leg]:
                    engine.touched[leg] = True

        if (1 / ENGINE_FPS) - (time.time() - timer) > EPSILON:
            time.sleep((1 / ENGINE_FPS) - (time.time() - timer))

    # if some legs haven't touched ground yet, stop and try to reach it
    if WALKING_TERRAIN_ADAPTATION_ENABLED:
        # iterate until every leg has touched ground
        counter = 0
        while False in engine.touched and not rospy.is_shutdown():
            for leg in range(NUMBER_OF_LEGS):
                # if a leg touches ground end the cycle for it by setting the touched flag
                if engine.leg_ground[leg]:
                    engine.touched[leg] = True
                # if a leg hasn't touched ground yet try to reach it by lowering it on ground
                if not engine.touched[leg]:
                    engine.pose[leg][2] -= 0.005
            apply_transform(engine)
            engine.compute_ik()
            time.sleep(0.02)

            counter += 1
            if 0.005 * counter > MAX_TOUCHDOWN_DEPTH_SEARCH:
                rospy.logerr("some legs didn't touch ground!")
                return


def stand_up(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_SIT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], start_pose[0], end_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], start_pose[1], end_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2],
                  start_pose[2] + (LEG_ADJ_HEIGHT / 6),
                  start_pose[2] + LEG_ADJ_HEIGHT,
                  end_pose[2] + LEG_ADJ_HEIGHT,
                  end_pose[2] + (LEG_ADJ_HEIGHT / 6),
                  end_pose[2]]]
        curve = bezier.Curve(nodes, degree=5)
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
        pos = asymmetrical_sigmoid(i / (TIME_STAND_UP * ENGINE_FPS)) * (- STAND_HEIGHT - SIT_HEIGHT)
        engine.final_pose[:, 2] += pos

        engine.final_body_abs_trasl[2] = - SIT_HEIGHT - pos  # inverted because when legs go down, robot goes up

        engine.compute_ik()
        time.sleep(0.05)


def sit(engine):
    curves = []
    for start_pose, end_pose in zip(engine.hw_pose.copy(), DEFAULT_POSE.copy()):
        nodes = [[start_pose[0], start_pose[0], start_pose[0], end_pose[0], end_pose[0], end_pose[0]],
                 [start_pose[1], start_pose[1], start_pose[1], end_pose[1], end_pose[1], end_pose[1]],
                 [start_pose[2],
                  start_pose[2] + (LEG_ADJ_HEIGHT / 6),
                  start_pose[2] + LEG_ADJ_HEIGHT,
                  end_pose[2] + LEG_ADJ_HEIGHT,
                  end_pose[2] + (LEG_ADJ_HEIGHT / 6),
                  end_pose[2]]]
        curve = bezier.Curve(nodes, degree=5)
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
        pos = asymmetrical_sigmoid(i / (TIME_SIT * ENGINE_FPS)) * (- STAND_HEIGHT - SIT_HEIGHT)
        engine.final_pose[:, 2] -= pos

        engine.final_body_abs_trasl[2] = STAND_HEIGHT + pos  # inverted because when legs go down, robot goes up

        engine.compute_ik()
        time.sleep(0.05)


def stand(engine):
    # level legs
    if DYNAMIC_TERRAIN_ADAPTATION_ENABLED:
        leg_correction(engine)
    engine.body_abs_trasl[2] = STAND_HEIGHT
    apply_transform(engine)
    engine.compute_ik()


def apply_transform(engine):
    # make sure the nearest on ground leg to the body is at the right height
    # STAND_HEIGHT is positive therefore a - is needed
    if AUTO_HEIGHT_ADJUSTMENT_ENABLED:
        max_h = max([engine.pose[leg][2] for leg in range(NUMBER_OF_LEGS) if engine.touched[leg]])
        engine.pose[:, 2] -= max_h + STAND_HEIGHT
        # rospy.loginfo(str(max_h + STAND_HEIGHT))

    pose = engine.pose.copy()

    # check if legs on ground
    engine.leg_ground = engine.filtered_force_sensors > TOUCHDOWN_TRESHOLD

    # dynamic terrain adaptation (still work in progress)
    if DYNAMIC_TERRAIN_ADAPTATION_ENABLED:
        pose[:, 2] += engine.leg_z_correction

    # body pose commands
    pose = rotate(pose, engine.body_rot, inverse=True)
    pose -= engine.body_trasl

    # make sure the body is at the right angle
    pose_correction(engine)
    pose = rotate(pose, [engine.roll_correction, engine.pitch_correction, 0])

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
    # P controller mode
    # for leg in range(NUMBER_OF_LEGS):
    #     engine.leg_z_correction[leg] -= engine.leg_adapt_filter_val * (engine.leg_adapt_var_filtered_setpoint - engine.force_sensors[leg])
    #     engine.leg_z_correction[leg] = limit(engine.leg_z_correction[leg],  MAX_LEG_CORRECTION, -  MAX_LEG_CORRECTION)  # to make sure it doesn't grow without control
    # avg = float(sum(engine.leg_z_correction) / NUMBER_OF_LEGS)
    # engine.leg_z_correction -= avg

    # spring-damper mode
    n = engine.filtered_force_sensors
    L = STAND_HEIGHT
    s = SPRINGINESS
    d = DAMPINESS

    y = (engine.pose[:, 2] + L + (n / s))
    y_1 = engine.leg_z_prev_pos
    engine.leg_z_correction = d * y + ((1 - d) * y_1)

    # simulink
    # engine.leg_z_correction = engine.simulink_leg_z_correction

    # print(f"{engine.leg_z_correction} -- {engine.simulink_leg_z_correction}")

    engine.leg_z_prev_pos = engine.leg_z_correction


def sleep(engine):
    engine.final_pose = DEFAULT_SIT_POSE.copy()
    engine.body_abs_trasl[2] = - SIT_HEIGHT

    engine.compute_ik()
