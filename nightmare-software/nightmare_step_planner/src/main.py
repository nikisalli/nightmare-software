#!/usr/bin/env python3

from dataclasses import dataclass
import numpy as np

# ros imports
import rospy

from nightmare_step_planner.msg import command  # pylint: disable=no-name-in-module
from std_msgs.msg import Header, Int32, String

from nightmare_math.math import (rotation_matrix)

from nightmare_config.config import (DEFAULT_POSE,
                                     GAIT_TRIPOD)


@dataclass
class substep():
    leg: int
    pos: np.ndarray([0, 0, 0])


class stepPlannerNode():
    def __init__(self):
        self.state = 'sleep'  # actual engine state
        self.gait = 'tripod'
        self.prev_state = 'sleep'  # previous engine state
        self.body_displacement = [0] * 6
        self.walk_direction = [0] * 3
        self.steps = []
        self.gait_step = 0
        self.step_id = 0
        self.engine_step = 0

    def run(self):
        pub_state = rospy.Publisher("/nightmare/footsteps", String, queue_size=1)
        header = Header()

        while not rospy.is_shutdown():
            if self.engine_step > self.step_id - 1:
                self.generate_next_steps()

                header.stamp = rospy.Time.now()
                pub_state.publish(String('json'))

    def set_state(self, msg):
        self.state = msg.state
        self.walk_direction = msg.walk_direction
        self.body_displacement = msg.body_displacement
        self.gait = msg.gait

    def set_engine_step(self, msg):
        self.engine_step = msg.data

    def generate_next_steps(self):
        if self.gait == 'tripod':
            step = []
            for leg in GAIT_TRIPOD[self.gait_step]:
                step.append(substep(
                    leg=leg,
                    pos=np.array([0, 0, 0])
                ))

                """step = DEFAULT_POSE[leg]
                step[0] += self.walk_direction[0]
                step[1] += self.walk_direction[1]
                step = rotation_matrix(step, [0, 0, self.walk_direction[2]])
                step.append(self.step_id)
                self.steps.append(step)"""

            self.step_id += 1
            self.gait_step += 1
            if self.gait_step == len(GAIT_TRIPOD):
                self.gait_step = 0

            rospy.loginfo(f"planner step: {self.step_id} planner gait step: {self.gait_step} gait: {self.gait}")


if __name__ == '__main__':
    rospy.init_node('step_planner')

    rospy.loginfo("starting step planner")
    engine = stepPlannerNode()

    rospy.loginfo("subscribing to nodes")
    rospy.Subscriber("/nightmare/command", command, engine.set_state)
    rospy.Subscriber("/engine/step", Int32, engine.set_state)

    engine.run()
