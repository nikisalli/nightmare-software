#!/usr/bin/env python3
# pylint: disable=broad-except, no-name-in-module

# libs import
import os
import sys
import json
import numpy as np
import time

# ros libs import
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int32, String, Float32, Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros as tf

# external package import
from nightmare_config.config import (DEFAULT_POSE,
                                     ENGINE_FPS,
                                     ENGINE_REFERENCE_FRAME,
                                     ENGINE_OUTPUT_ODOM_TOPIC,
                                     POSTURE_P_GAIN,
                                     LEG_P_GAIN,
                                     LEG_SETPOINT_FILTER_VAL)

from nightmare_state_broadcaster.msg import command
from nightmare_math.math import abs_ang2pos, abs_pos2ang, euler2quat

# same package import
sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from modules import movements


JOINTSTATE_MSG = JointState(header=Header(),
                            name=['leg1coxa', 'leg1femur', 'leg1tibia',
                                  'leg2coxa', 'leg2femur', 'leg2tibia',
                                  'leg3coxa', 'leg3femur', 'leg3tibia',
                                  'leg4coxa', 'leg4femur', 'leg4tibia',
                                  'leg5coxa', 'leg5femur', 'leg5tibia',
                                  'leg6coxa', 'leg6femur', 'leg6tibia', 'tail_joint'],
                            velocity=[],
                            effort=[])


class engineNode():
    def __init__(self):
        # ROBOT COMMANDS #
        self.state = 'sleep'                    # actual engine state
        self.prev_state = 'sleep'               # previous engine state
        self.gait = 'tripod'
        self.body_trasl = np.array([0, 0, 0])   # commanded body traslation
        self.body_rot = np.array([0, 0, 0])     # commanded body rotation
        self.walk_trasl = np.array([0, 0, 0])   # commanded walk traslatory speed
        self.walk_rot = np.array([0, 0, 0])     # commanded walk rotatory speed

        # ROBOT STATE #
        self.angles = np.zeros(shape=(6, 3))    # angle matrix
        self.angles_array = [0] * 19            # angle array
        self.hw_angles_array = [0] * 19
        self.hw_angles = np.zeros(shape=(6, 3))
        self.states = [0] * 19                  # all servos disconnected at start
        self.hw_pose = np.zeros(shape=(6, 3))   # body frame initialize as empty and fill it later in set_hw_joint_state callback
        self.pose = DEFAULT_POSE.copy()         # body frame pose of the robot without transforms  6 x 3
        self.final_pose = DEFAULT_POSE.copy()   # body frame pose of the robot with transforms  6 x 3
        self.rate = rospy.Rate(ENGINE_FPS)
        self.body_abs_trasl = [0, 0, 0]         # engine reference frame
        self.body_abs_rot = [0, 0, 0]           # engine reference frame
        self.final_body_abs_trasl = [0, 0, 0]   # engine reference frame
        self.final_body_abs_rot = [0, 0, 0]     # engine reference frame
        self.leg_ground = np.array([True] * 6)  # array of bools True if leg touching ground
        self.filtered_force_sensors = np.asarray([0.] * 6)
        self.force_sensors = np.asarray([0.] * 6)

        # POSTURE CORRECTION #
        self.pitch_correction = 0               # process value for pose pitch P filter
        self.roll_correction = 0                # process value for pose roll P filter
        self.pose_filter_val = POSTURE_P_GAIN   # pose P filter aggressivity
        self.pose_pitch_setpoint = 0
        self.pose_roll_setpoint = 0

        # TERRAIN ADAPTATION #
        self.leg_z_correction = np.array([0] * 6)     # leg filters process variables
        self.simulink_leg_z_correction = np.array([0] * 6)
        self.leg_z_speed = np.array([0] * 6)
        self.leg_z_prev_pos = np.array([0] * 6)
        self.leg_z_prev_time = time.time()
        self.init_time = time.time()
        # P controller
        self.leg_adapt_filter_val = LEG_P_GAIN           # leg P gain
        self.leg_adapt_var_filtered_setpoint = 0         # leg filters' setpoint filter process variable
        self.leg_adapt_var_setpoint = 0                  # leg filters' setpoint filter setpoint

        # STEP PLANNER STATE #
        self.step_id = 0
        self.steps = []  # world frame
        self.attenuation = 0

        # create publishers
        self.joint_angle_publisher = rospy.Publisher('/engine/angle_joint_states', JointState, queue_size=10)
        self.joint_angle_msg = JOINTSTATE_MSG   # joint angle topic structure

        self.joint_state_publisher = rospy.Publisher('/engine/state_joint_states', JointState, queue_size=10)
        self.joint_state_msg = JOINTSTATE_MSG   # joint state topic structure

        self.engine_step_id_publisher = rospy.Publisher('/engine/step', Int32, queue_size=10)
        self.engine_step_id_msg = Int32()       # engine current step id

        self.marker_publisher = rospy.Publisher("/engine/markers", Marker, queue_size=100)
        self.robotMarker = Marker()             # engine markers

        self.odom_pub = rospy.Publisher(ENGINE_OUTPUT_ODOM_TOPIC, Odometry, queue_size=50)
        self.odom_pub_msg = Odometry()          # engine odom

        # TF LISTENER #
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # setup body to map odometry
        self.odom_pub_msg.header.frame_id = ENGINE_REFERENCE_FRAME  # odom
        self.odom_pub_msg.child_frame_id = "base_link"

        # setup marker
        self.robotMarker.header.frame_id = ENGINE_REFERENCE_FRAME
        self.robotMarker.ns = "engine"
        self.robotMarker.id = 0
        self.robotMarker.type = 2  # sphere
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0
        self.robotMarker.pose.position.y = 0
        self.robotMarker.pose.position.z = 0
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 0.01
        self.robotMarker.scale.y = 0.01
        self.robotMarker.scale.z = 0.01

        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

        self.robotMarker.lifetime = 5.0

        self.red = [1, 0, 0]
        self.green = [0, 1, 0]
        self.blue = [0, 0, 1]

    def compute_ik(self):
        t = rospy.Time.now()
        self.angles_array = abs_pos2ang(self.final_pose)

        self.publish_states(t)
        self.publish_joints(t)  # publish engine output pose
        self.publish_engine_odom(t)  # publish engine transform for enhanced slam and odometry

    def run(self):
        rospy.wait_for_message("/joint_states", JointState)  # at start wait for first hardware frame to know where to start
        rospy.wait_for_message("/nightmare/command", command)

        while not rospy.is_shutdown():
            # if changing sit or standup and adjust leg positions
            if self.state == 'stand' and self.prev_state == 'sleep':
                self.states = [1] * 19
                movements.stand_up(self)
            elif self.state == 'sleep' and (self.prev_state == 'stand' or any(v != 0 for v in self.states)):
                movements.sit(self)
                self.states = [0] * 19
            elif self.state == 'stand':
                self.states = [1] * 19
                if len(self.steps) > 0:  # if there are steps to execute, execute them
                    movements.execute_step(self)
                else:
                    movements.stand(self)
            elif self.state == 'sleep':
                movements.sleep(self)
                self.states = [0] * 19

            self.prev_state = self.state
            self.rate.sleep()

    def parse_footsteps(self, msg):
        self.steps = json.loads(msg.data)
        # rospy.loginfo('\n' + str(engine.steps) + '\n')

    def publish_engine_odom(self, t):
        # transformation
        x = self.final_body_abs_trasl[0]
        y = self.final_body_abs_trasl[1]
        z = self.final_body_abs_trasl[2]
        q = euler2quat([self.final_body_abs_rot[0], self.final_body_abs_rot[1], self.final_body_abs_rot[2]])

        # odom
        self.odom_pub_msg.header.stamp = t
        self.odom_pub_msg.pose.pose = Pose(Point(x, y, z), Quaternion(*q))
        self.odom_pub_msg.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.odom_pub.publish(self.odom_pub_msg)

    def publish_joints(self, t):
        self.joint_angle_msg.position = self.angles_array
        self.joint_angle_msg.header.stamp = t
        self.joint_angle_publisher.publish(self.joint_angle_msg)

    def publish_states(self, t):
        self.joint_state_msg.position = self.states
        self.joint_state_msg.header.stamp = t
        self.joint_state_publisher.publish(self.joint_state_msg)

    def publish_step_id(self):
        self.engine_step_id_msg.data = self.step_id
        self.engine_step_id_publisher.publish(self.engine_step_id_msg)

    def set_attenuation(self, msg):
        self.attenuation = msg.data

    def set_force_sensors(self, msg):
        self.force_sensors = np.asarray(list(msg.data))

        # update terrain adaptation setpoint
        self.leg_adapt_var_setpoint = sum(self.force_sensors) / len(self.force_sensors)  # average
        self.leg_adapt_var_filtered_setpoint += LEG_SETPOINT_FILTER_VAL * (self.leg_adapt_var_setpoint - self.leg_adapt_var_filtered_setpoint)

    def set_simulink_leg_correction(self, msg):
        self.simulink_leg_z_correction = np.asarray(list(msg.data))

    def set_filtered_force_sensors(self, msg):
        self.filtered_force_sensors = np.asarray(list(msg.data))

    def set_state(self, msg):
        self.body_trasl = np.array(msg.body_trasl)
        self.body_rot = np.array(msg.body_rot)
        self.walk_trasl = np.array(msg.walk_trasl)
        self.walk_rot = np.array(msg.walk_rot)
        self.state = msg.state
        self.gait = msg.gait

    def set_hw_joint_state(self, msg):
        self.hw_angles_array = msg.position
        self.hw_angles = np.reshape(self.hw_angles_array[:-1], newshape=(6, 3))
        self.hw_pose = abs_ang2pos(self.hw_angles_array)

    def publish_marker(self, pos, life, color):
        self.robotMarker.id += 1

        self.robotMarker.pose.position.x = pos[0]
        self.robotMarker.pose.position.y = pos[1]
        self.robotMarker.pose.position.z = pos[2]

        self.robotMarker.color.r = color[0]
        self.robotMarker.color.g = color[1]
        self.robotMarker.color.b = color[2]

        self.robotMarker.lifetime = rospy.Duration(life)

        self.marker_publisher.publish(self.robotMarker)

    def publish_pose_markers(self, pose, life, color):
        for pos in pose:
            self.publish_marker(pos, life, color)


if __name__ == '__main__':
    rospy.init_node('movement_engine')

    rospy.loginfo("starting engine node")
    engine = engineNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)
    rospy.Subscriber("/joint_states", JointState, engine.set_hw_joint_state)
    rospy.Subscriber("/engine/footsteps", String, engine.parse_footsteps)
    rospy.Subscriber("/engine/attenuation", Float32, engine.set_attenuation)
    rospy.Subscriber("/force_sensors", Float32MultiArray, engine.set_force_sensors)
    rospy.Subscriber("/filtered_force_sensors", Float32MultiArray, engine.set_filtered_force_sensors)

    # for simulink simulation #TODO remove
    rospy.Subscriber("/simulink/leg_correction", Float32MultiArray, engine.set_simulink_leg_correction)

    engine.run()
