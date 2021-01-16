#!/usr/bin/env python3

import json
import time
import numpy as np
from numpy import sqrt
from scipy.spatial.transform import Rotation as R

# ros imports
import rospy
import tf2_ros as tf
from nightmare_step_planner.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import Header, Int32, String, Float32
from visualization_msgs.msg import Marker

from nightmare_math.math import euler_rotation_matrix
from nightmare_config.config import GAIT, LEG_TIPS, DEFAULT_POSE, PI, MAX_STEP_LENGTH


class stepPlannerNode():
    def __init__(self):
        # robot state
        self.state = 'sleep'  # actual engine state
        self.gait = 'tripod'
        self.prev_state = 'sleep'  # previous engine state
        self.body_displacement = [0] * 6
        self.walk_direction = [0] * 3

        # step planner state
        self.steps = []
        self.gait_step = 0
        self.step_id = 0
        self.engine_step = 0
        self.attenuation = 0

        # body state
        self.abs_body_pose = np.zeros(shape=(6, 3))
        self.body_trans = 0
        self.body_np_trans = np.ndarray((4, 4))  # 4x4 transform matrix to switch frame

        # tf listener
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(60)

        # marker settings
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world"
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

        # publishers
        self.header = Header()
        self.pub_steps = rospy.Publisher("/engine/footsteps", String, queue_size=1)
        self.marker_publisher = rospy.Publisher("/engine/markers", Marker, queue_size=100)
        self.engine_attenuation_publisher = rospy.Publisher('/engine/attenuation', Float32, queue_size=10)
        self.engine_attenuation_msg = Float32()  # engine current step id

    def run(self):
        while not rospy.is_shutdown():
            self.parse_tf()
            mod = sqrt(self.walk_direction[0]**2 + self.walk_direction[1]**2)
            if (self.engine_step >= self.step_id - (len(GAIT[self.gait]) / 2) and self.state == 'stand' and (abs(mod) > 0.01 or abs(self.walk_direction[2]) > 0.01)):
                # check if:
                # - a step is already present
                # - check if the robot is standing to start walking
                # - check if a command to walk is valid
                self.generate_next_steps()
            self.rate.sleep()

    def generate_next_steps(self):
        t = time.time()

        self.step_id += 1
        step = []

        dpose = euler_rotation_matrix(DEFAULT_POSE, [0, 0, PI / 2])
        translation = [self.body_trans.translation.x, self.body_trans.translation.y, self.body_trans.translation.z]
        rotation = [self.body_trans.rotation.x, self.body_trans.rotation.y, self.body_trans.rotation.z, self.body_trans.rotation.w]
        rotation_matrix = R.from_quat(rotation)
        rotation_euler = rotation_matrix.as_euler('xyz')
        inverse_rotation_matrix = rotation_matrix.inv()
        abs_target_pose = rotation_matrix.apply(dpose)
        abs_target_pose += translation

        self.publish_pose_markers(abs_target_pose, 1, self.green)

        # find absolute max displacement
        npose = abs_target_pose.copy()
        npose += euler_rotation_matrix([self.walk_direction[0] * 0.5, self.walk_direction[1] * 0.5, 0], [0, 0, rotation_euler[2]])
        npose -= translation
        npose = euler_rotation_matrix(npose, [0, 0, - self.walk_direction[2] * 0.5])
        npose = inverse_rotation_matrix.apply(npose)
        tr = np.subtract(npose, dpose)
        displ = max([sqrt(leg[0]**2 + leg[1]**2 + leg[2]**2) for leg in tr])

        # generate a coefficient to scale the step so that is stays in the defined limits
        if displ > MAX_STEP_LENGTH:
            self.attenuation = MAX_STEP_LENGTH / displ
        else:
            self.attenuation = 1

        # generate steps
        for leg in GAIT[self.gait][self.gait_step]:
            prev_pos = abs_target_pose[leg]
            new_pos = prev_pos.copy()

            #  =======================
            # rotate command with the body or you end up with messed matrices!
            comtrans = euler_rotation_matrix([self.walk_direction[0] * self.attenuation * 0.5, self.walk_direction[1] * self.attenuation * 0.5, 0], [0, 0, rotation_euler[2]])
            new_pos += comtrans
            abs_trans = comtrans + translation

            # can't be bothered TODO
            new_pos -= translation
            new_pos = euler_rotation_matrix(new_pos, [0, 0, - self.walk_direction[2] * self.attenuation * 0.5])
            new_pos += translation
            #  =======================

            self.publish_marker(new_pos, 1, self.red)

            new_pos -= translation
            new_pos = inverse_rotation_matrix.apply(new_pos)
            trans = new_pos - dpose[leg]

            step.append({'leg': int(leg), 'pos': trans.tolist()})

        self.steps.append({'id': int(self.step_id), 'trans': abs_trans.tolist(), 'steps': step})
        self.gait_step += 1
        self.publish_attenuation()

        if self.gait_step == len(GAIT[self.gait]):  # check if gait cycle got to the end
            self.gait_step = 0

        self.header.stamp = rospy.Time.now()
        self.pub_steps.publish(String(json.dumps(self.steps)))

        rospy.loginfo(f"generation took: {time.time() - t}s planner_step: {self.step_id} planner_gait_step: {self.gait_step} engine_step: {self.engine_step}")

    def set_state(self, msg):
        self.state = msg.state
        self.walk_direction = msg.walk_direction
        self.body_displacement = msg.body_displacement
        self.gait = msg.gait

    def set_engine_step(self, msg):
        self.engine_step = msg.data
        self.steps.pop(0)

    def parse_tf(self):
        try:
            # get leg tip absolute transform
            for i, leg in enumerate(LEG_TIPS):
                trans = self.tf_buffer.lookup_transform('world', leg, rospy.Time(0), rospy.Duration(.1)).transform.translation
                self.abs_body_pose[i] = [trans.x, trans.y, trans.z]

            # get body transform
            trans = self.tf_buffer.lookup_transform('world', 'body_link', rospy.Time(0), rospy.Duration(.1)).transform
            self.body_trans = trans

            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            return 0

    def publish_attenuation(self):
        self.engine_attenuation_msg.data = self.attenuation
        self.engine_attenuation_publisher.publish(self.engine_attenuation_msg)

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
    rospy.init_node('step_planner')

    rospy.loginfo("starting step planner")
    engine = stepPlannerNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)
    rospy.Subscriber("/engine/step", Int32, engine.set_engine_step)

    engine.run()
