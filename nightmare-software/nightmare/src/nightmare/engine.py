import numpy as np
import sys
from dataclasses import dataclass
from numpy import sin, cos, arccos, arctan2, sqrt
from enum import Enum
import typing
from threading import Semaphore

# ros imports
import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension

# module imports
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal
from nightmare.modules.math import no_zero, asymmetrical_sigmoid
from nightmare import config
from nightmare.config import PI, EPSILON
from nightmare.msg import command


# utils
def time_s():
    '''get rospy time in seconds'''
    return rospy.get_rostime().to_sec()


# data classes
class Pose:
    body_pos: np.ndarray(shape=(6, 3))
    enables: np.ndarray(shape=6)

    def __init__(self, body_pos: np.ndarray(shape=(6, 3)), enables: np.ndarray(shape=6)):
        self.body_pos = body_pos
        self.enables = enables

    def is_near(self, other) -> bool:
        '''check if poses are near'''
        return np.all(np.abs(self.body_pos - other.body_pos) < config.POSE_NEAR_THRESHOLD)

    def __str__(self):
        return f'Pose(body_pos={self.body_pos}, enables={self.enables})'


class RobotState:
    pose: Pose = Pose(np.zeros(shape=(6, 3)), np.ones(shape=6))
    cmd: command = command()
    force_sensors: np.ndarray(shape=6) = np.zeros(shape=6)
    averaged_force_sensors: np.ndarray(shape=6) = np.zeros(shape=6)

    def __str__(self):
        return f'''RobotState:\n\tpose: {self.pose}\n
                   \tcommand: {self.cmd}\n
                   \tforce_sensors: {self.force_sensors}\n
                   \taveraged_force_sensors: {self.averaged_force_sensors}'''


# FSM states
class IdleState:
    '''returns same pose as start and disables all servos'''
    _idle_pose: Pose

    def __init__(self, state: RobotState):
        # get pose to return
        self._idle_pose = Pose(state.pose.body_pos, np.zeros(shape=6))

    def update(self, state: RobotState) -> (typing.Any, Pose):
        if state.cmd.state == 'idle':
            return self, self._idle_pose
        elif state.cmd.state == 'awake':
            return AdjustGetUpState(state), self._idle_pose
        else:
            perr(f'unhandled state in idle state: {state}')


class AdjustGetUpState:
    '''adjust legs to get up and calibrate load cells'''
    _start_pose: Pose
    _start_time: float

    def __init__(self, state: RobotState):
        self._start_pose = state.pose
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_GET_UP_LEG_ADJ
        if advancement < 1:
            return self, Pose(self._start_pose.body_pos + (config.DEFAULT_SIT_POSE - self._start_pose.body_pos) * advancement, np.ones(shape=6))
        else:
            # calibrate force sensors
            rospy.set_param("/hardware/load_cells_offsets", state.force_sensors.tolist())
            return GetUpState(state), state.pose


class GetUpState:
    '''get up'''
    _start_time: float

    def __init__(self, state: RobotState):
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_GET_UP
        if advancement < 1:
            return self, Pose(config.DEFAULT_SIT_POSE + (config.DEFAULT_POSE - config.DEFAULT_SIT_POSE) * advancement, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'idle':
            return SitState(state), Pose(state.pose.body_pos, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'awake' and state.cmd.mode == 'stand':
            return StandState(state), Pose(state.pose.body_pos, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'awake' and state.cmd.mode == 'walk':
            return WalkState(state), Pose(state.pose.body_pos, np.ones(shape=6))
        else:
            perr(f'unhandled state in get up state: {state}')


class SitState:
    '''sit'''
    _start_time: float

    def __init__(self, state: RobotState):
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_SIT
        if advancement < 1:
            return self, Pose(config.DEFAULT_POSE + (config.DEFAULT_SIT_POSE - config.DEFAULT_POSE) * advancement, np.ones(shape=6))
        else:
            return IdleState(state), Pose(state.pose.body_pos, np.ones(shape=6))


"""class AdjustSitState:
    '''adjust legs before sitting'''
    _start_time: float

    def __init__(self, state: RobotState):
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_SIT_LEG_ADJ"""


class FiniteStateMachine:
    _state: typing.Any

    def __init__(self, state: RobotState):
        self._state = IdleState(state)

    def update(self, state: RobotState) -> Pose:
        '''get next state and updated pose'''
        self._state, pose = self._state.update(state)
        return pose


class EngineNode:
    _tf_listener: tf.TransformListener
    _cmd_subscriber: rospy.Subscriber
    _force_sensors_subscriber: rospy.Subscriber
    _engine_joint_publisher: rospy.Publisher
    _engine_joint_publisher_msg: JointState
    _robot_state: RobotState
    _state_machine: FiniteStateMachine

    def __init__(self):
        # hardware robot state constantly updated by the callbacks
        self._robot_state = RobotState()

        # subscribers
        self._tf_listener = tf.TransformListener()
        self._cmd_subscriber = rospy.Subscriber("/control/command", command, self.command_callback)
        self._force_sensors_subscriber = rospy.Subscriber("/hardware/load_cells", Float32MultiArray, self.force_sensors_callback)

        # publishers
        self._engine_joint_publisher_msg = JointState(header=Header(), name=config.JOINT_STATE_LABELS)
        self._engine_joint_publisher = rospy.Publisher('/engine/joint_states', JointState, queue_size=10)

        # wait for first transform to start
        pinfo("waiting for first transform...")
        self._tf_listener.waitForTransform('/body_link', 'leg_1_coxa_1', rospy.Time(0), rospy.Duration(100000000))
        pinfo("waiting for first command...")
        rospy.wait_for_message("/control/command", command)

        # state machine
        self._finite_state_machine = FiniteStateMachine(self._robot_state)

        # initialize hardware pose
        self.get_hardware_pose()

        pinfo("ready")

    def set_hardware_pose(self, pose: Pose):
        '''compute ik and write angles to hardware with current enables'''
        def relative_ik(rel_pos, leg_dim: np.ndarray(shape=(6, 3))) -> np.ndarray(shape=(3,)):
            x, y, z = rel_pos
            CX, FM, TB = leg_dim

            # position validity check
            coxa_director = np.array([x, y, 0]) / sqrt(x ** 2 + y ** 2)
            coxa_tip = coxa_director * CX
            tip_to_coxa_dist = np.linalg.norm(rel_pos - coxa_tip)
            director = (rel_pos - coxa_tip) / tip_to_coxa_dist
            if tip_to_coxa_dist > FM + TB:
                pwarn("position is not reachable! (too far) finding a possible one...")
                x, y, z = coxa_tip + (FM + TB - EPSILON) * director
            elif tip_to_coxa_dist < abs(FM - TB):
                pwarn("position is not reachable! (too close) finding a possible one...")
                x, y, z = coxa_tip + (abs(FM - TB) + EPSILON) * director

            d1 = sqrt(y**2 + x**2) - CX
            d = sqrt(z**2 + (d1)**2)
            alpha = -arctan2(y, x)
            beta = arccos((z**2 + d**2 - d1**2) / (2 * (- no_zero(z)) * d)) + arccos((FM**2 + d**2 - TB**2) / (2 * FM * d))
            gamma = - arccos((FM**2 + TB**2 - d**2) / (2 * FM * TB)) + 2 * PI
            return np.array([alpha, beta - PI / 2, gamma - (PI / 2) * 3])

        rel_poses = (pose.body_pos - config.POSE_OFFSET) * config.POSE_REL_CONVERT
        angles = np.ravel(np.array([relative_ik(rel, leg.dim) for rel, leg in zip(rel_poses, config.legs)]))
        # publish the angles
        self._engine_joint_publisher_msg.position = angles
        self._engine_joint_publisher_msg.effort = np.ravel(np.tile(pose.enables, (3, 1)).transpose((1, 0)))  # expand leg enable to its 3 joints
        self._engine_joint_publisher_msg.header.stamp = rospy.Time.now()
        self._engine_joint_publisher.publish(self._engine_joint_publisher_msg)

    def get_hardware_pose(self):
        '''get current robot pose from hardware through tf'''
        try:
            for i, leg_tip in enumerate(config.LEG_TIPS):
                self._robot_state.pose.body_pos[i] = np.array(self._tf_listener.lookupTransform('/body_link', leg_tip, rospy.Time(0))[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pwarn("couldn't get transform!")
            raise TimeoutError("couldn't get transform!")

    def command_callback(self, msg):
        self._robot_state.cmd = msg

    def force_sensors_callback(self, msg):
        self._robot_state.force_sensors = np.array(msg.data)

    def update(self):
        self.get_hardware_pose()
        self.set_hardware_pose(self._finite_state_machine.update(self._robot_state))
        # self._state_machine.update(self._current_command, self._current_command)

        # self.get_hardware_pose()
        # self.set_hardware_pose(self._body_pos)


if __name__ == '__main__':
    rospy.init_node('engine')

    pinfo("starting")

    node = EngineNode()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    pinfo("stopped")
