#!/usr/bin/env python3

import json
import numpy as np
from numpy import sqrt

# ros imports
import rospy
import tf2_ros as tf
from nightmare_step_planner.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import (Header,
                          Int32,
                          String)

from visualization_msgs.msg import Marker

from nightmare_math.math import (rotation_matrix)

from nightmare_config.config import (GAIT_TRIPOD,
                                     leg_tips)


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

        # body state
        self.abs_body_pose = np.zeros(shape=(6, 3))
        self.body_pos = np.array([0, 0, 0])

        # tf listener
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(10)

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
        self.pub_steps = rospy.Publisher("/engine/footsteps", String, queue_size=1)
        self.marker_publisher = rospy.Publisher("/engine/markers", Marker, queue_size=100)

    def run(self):
        header = Header()

        while not rospy.is_shutdown():
            mod = sqrt(self.walk_direction[0]**2 + self.walk_direction[1]**2)
            if (self.engine_step > self.step_id - 1 and self.state == 'stand' and (abs(mod) > 0.01 or self.walk_direction[2] > 1)):
                # check if:
                # - a step is already present
                # - check if the robot is standing to start walking
                # - check if a command to walk is valid

                if not self.parse_tf():
                    break
                self.generate_next_steps()

                header.stamp = rospy.Time.now()
                self.pub_steps.publish(String(json.dumps(self.steps)))

            self.rate.sleep()

    def generate_next_steps(self):
        if self.gait == 'tripod':
            step = []
            for leg in GAIT_TRIPOD[self.gait_step]:
                prev_pos = self.abs_body_pose[leg]
                new_pos = np.array([prev_pos[0] + self.walk_direction[0], prev_pos[1] + self.walk_direction[1], 0.0])
                new_pos = rotation_matrix(new_pos, [0, 0, self.walk_direction[2]])
                step.append({'leg': int(leg), 'pos': new_pos.tolist()})
                self.publish_marker(new_pos, -1, self.red)

            self.steps.append(step)
            self.step_id += 1
            self.gait_step += 1

            if self.gait_step == len(GAIT_TRIPOD):  # check if gait cycle got to the end
                self.gait_step = 0

            rospy.loginfo(f"planner step: {self.step_id} planner gait step: {self.gait_step} gait: {self.gait}")

    def set_state(self, msg):
        self.state = msg.state
        self.walk_direction = msg.walk_direction
        self.body_displacement = msg.body_displacement
        self.gait = msg.gait

    def set_engine_step(self, msg):
        self.engine_step = msg.data

    def parse_tf(self):
        try:
            for i, leg in enumerate(leg_tips):
                trans = self.tf_buffer.lookup_transform('world', leg, rospy.Time.now(), rospy.Duration(3.0)).transform.translation
                self.abs_body_pose[i] = [trans.x, trans.y, trans.z]
            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            return 0

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


if __name__ == '__main__':
    rospy.init_node('step_planner')

    rospy.loginfo("starting step planner")
    engine = stepPlannerNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)
    rospy.Subscriber("/engine/step", Int32, engine.set_state)

    engine.run()
