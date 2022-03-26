import numpy as np
from numpy import sin, cos, arccos, arctan2, sqrt
import typing

# ros imports
import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32, Float32MultiArray, MultiArrayDimension

# module imports
from nightmare.modules.logging import printlog, loglevel, pinfo, pwarn, perr, pfatal
from nightmare.modules.math import no_zero, asymmetrical_sigmoid, rotate, vmult, shortest_distance_two_segments_2d
from nightmare import config
from nightmare.config import PI, EPSILON
from nightmare.msg import command
from nightmare.modules.bezier import Bezier as bezier

from nightmare.modules.debug import plot


# utils
def time_s():
    '''get rospy time in seconds'''
    return rospy.get_rostime().to_sec()


# data classes
class Pose:
    body_pos: np.ndarray(shape=(6, 3))
    enables: np.ndarray(shape=6)

    def __init__(self, body_pos: np.ndarray(shape=(6, 3)),
                 enables: np.ndarray(shape=6)):
        self.body_pos = body_pos.copy()
        self.enables = enables.copy()

    def __str__(self):
        return f'Pose(body_pos={self.body_pos}, enables={self.enables})'

    def is_near(self, other) -> bool:
        '''check if poses are near'''
        return np.all(np.abs(self.body_pos - other.body_pos) < config.POSE_NEAR_THRESHOLD)

    def rotate(self, euler_angles: np.ndarray(shape=(3,)),
               mask: np.ndarray(shape=(6,)) = np.full((6,), True),
               pivot: np.ndarray(shape=(3,)) = None,
               inverse: bool = False):
        '''rotate the pose around a pivot point'''
        # self.body_pos = rotate(self.body_pos, euler_angles, pivot, inverse, mask)
        temp = self.body_pos.copy()
        self.body_pos = vmult(rotate(self.body_pos, euler_angles, pivot, inverse), mask) + vmult(temp, np.invert(mask))
        return self

    def translate(self, translation: np.ndarray(shape=(3,)),
                  mask: np.ndarray(shape=(6,)) = np.full((6,), True)):
        '''translate the pose'''
        # self.body_pos += translation
        self.body_pos += vmult(np.tile(translation, (6, 1)), mask)  # sum only non masked vectors
        return self

    def bezier(self, t: float,
               poses: list,
               mask: np.ndarray(shape=(6,)) = np.full((6,), True)):
        '''move every leg along a bezier curve interpolated on multiple poses
        t: parameter in [0, 1]
        poses: list of poses to interpolate between
        mask: mask to apply to not interpolate certain legs

        Note: this does not work with disabled legs, every leg must be enabled
        '''
        points = [p.body_pos for p in poses]
        temp = self.body_pos.copy()
        self.body_pos = vmult(bezier.Point(t, points), mask) + vmult(temp, np.invert(mask))
        return self

    def mask(self, pose,
             mask: np.ndarray(shape=(6,))):
        '''mask the pose with another pose'''
        self.body_pos = vmult(pose.body_pos, mask) + vmult(self.body_pos, np.invert(mask))
        return self

    def copy(self):
        return Pose(self.body_pos.copy(), self.enables.copy())

    def disable(self):
        self.enables = np.zeros(6)
        return self


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
    def __init__(self, state: RobotState, pose: Pose):
        pass

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        if state.cmd.state == 'idle':
            return self, state.pose.disable()
        elif state.cmd.state == 'awake':
            return AdjustGetUpState(state, pose), state.pose.disable()
        else:
            perr(f'unhandled state in idle state: {state}')


class AdjustGetUpState:
    _start_pose: Pose
    _start_time: float

    def __init__(self, state: RobotState, pose: Pose):
        self._start_pose = state.pose
        self._start_time = time_s()
        # print("leg adj start pose:\n", self._start_pose)

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_GET_UP_LEG_ADJ
        if advancement < 1:
            return self, Pose(self._start_pose.body_pos + (config.DEFAULT_SIT_POSE - self._start_pose.body_pos) * advancement,
                              np.ones(shape=6))  # enable all servos
        else:
            # calibrate force sensors
            rospy.set_param("/hardware/load_cells_offsets", state.force_sensors.tolist())
            return GetUpState(state, pose), Pose(config.DEFAULT_SIT_POSE, np.ones(shape=6))


class GetUpState:
    _start_time: float

    def __init__(self, state: RobotState, pose: Pose):
        self._start_time = time_s()
        # print("get up start pose:\n", state.pose)

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_GET_UP
        if advancement < 1:
            return self, Pose(config.DEFAULT_SIT_POSE + (config.DEFAULT_POSE - config.DEFAULT_SIT_POSE) * advancement, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'idle':
            return AdjustSitState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'awake' and state.cmd.mode == 'stand':
            return StandState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        elif advancement > 1 and state.cmd.state == 'awake' and state.cmd.mode == 'walk':
            return WalkState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        else:
            perr(f'unhandled state in get up state: {state}')


class SitState:
    _start_time: float

    def __init__(self, state: RobotState, pose: Pose):
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        advancement = self.task_time_s() / config.TIME_SIT
        if advancement < 1:
            return self, Pose(config.DEFAULT_POSE + (config.DEFAULT_SIT_POSE - config.DEFAULT_POSE) * advancement, np.ones(shape=6))
        else:
            return IdleState(state, pose), Pose(config.DEFAULT_SIT_POSE, np.ones(shape=6))


class AdjustSitState:
    _start_time: float

    def __init__(self, state: RobotState, pose: Pose):
        self._start_time = time_s()

    def task_time_s(self) -> float:
        return time_s() - self._start_time

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        return SitState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))


class StandState:
    _start_pose: Pose

    def __init__(self, state: RobotState, pose: Pose):
        self._start_pose = state.pose
        # print("stand start pose:\n", self._start_pose)

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        if state.cmd.state == 'awake' and state.cmd.mode == 'stand':
            return self, Pose(config.DEFAULT_POSE, np.ones(shape=6)).translate(state.cmd.body_trasl).rotate(state.cmd.body_rot)
        elif state.cmd.state == 'awake' and state.cmd.mode == 'walk':
            return WalkState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        elif state.cmd.state == 'idle':
            return AdjustSitState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        else:
            perr(f'unhandled state in stand state: {state}')


class WalkState:
    _gait_step: int  # step index from 0 to len(gait) - 1
    _gait_step_state: float  # 0 to 1
    _last_step_pose: Pose  # pose of the last step

    def __init__(self, state: RobotState, pose: Pose):
        self._gait_step = 0
        self._gait_step_state = 0
        self._last_step_pose = pose.copy()

    def update(self, state: RobotState, pose: Pose) -> (typing.Any, Pose):
        if (state.cmd.state == 'awake' and state.cmd.mode == 'walk') or self._gait_step_state != 0:
            # translate and rotate the current pose according to the walk command
            temp = pose.copy()
            # calculate max speed for the current step
            # get current gait
            gait = config.GAIT[state.cmd.gait]
            # get current step
            gait_step = gait[self._gait_step]

            # calculate reduction factor
            def cost(x: float) -> float:
                local_temp = temp.copy()
                # ##### PREDICTION
                # predict robot pose at the end of the step
                # legs on ground
                legs_on_ground_mask = np.invert(gait_step)
                total_mult_factor = x * 2 * len(gait) * (1 - self._gait_step_state)
                local_temp.translate(- state.cmd.walk_trasl * total_mult_factor, legs_on_ground_mask).rotate(- state.cmd.walk_rot * total_mult_factor, legs_on_ground_mask)
                # stepping legs
                legs_stepping_mask = gait_step
                total_mult_factor_step = x * config.STEP_TIME
                local_temp.mask(Pose(config.DEFAULT_POSE, np.ones((6,))).translate(state.cmd.walk_trasl * total_mult_factor_step).rotate(state.cmd.walk_rot * total_mult_factor_step), legs_stepping_mask)
                # calculate the distance from every leg tip
                dists = []
                for i in range(6):
                    for j in range(6):
                        if i != j:
                            # remove z component
                            p1a = local_temp.body_pos[i][:2]
                            p1b = config.POSE_OFFSET[i][:2]
                            p2a = local_temp.body_pos[j][:2]
                            p2b = config.POSE_OFFSET[j][:2]
                            min_segment_dist = shortest_distance_two_segments_2d(p1a, p1b, p2a, p2b)
                            dists.append(min_segment_dist)
                # get min distance
                dist = config.LEG_KEEPOUT_RADIUS - min(dists)
                if dist < 0:
                    return 0
                else:
                    return dist

            # ##### OPTIMIZATION
            red = 1  # default reduction factor
            # opt_steps = []
            for i in range(10):
                current_cost = cost(red)
                if current_cost < 0.01:
                    break
                if red < 0:
                    break

                # update red
                red -= 0.1
                # opt_steps.append([red, current_cost])

            # print(red)

            # debug optimization plot
            # reds = np.linspace(0, 2, 20)
            # costs = [cost(red) for red in reds]
            # plot(reds, costs, opt_steps)

            # translate and rotate legs on ground
            legs_on_ground_mask = np.invert(gait_step)
            total_mult_factor = red * (1 / config.ENGINE_FPS) * 2 * len(gait)
            # the minus sign is because the legs on ground need to move in the opposite direction to move the robot forward
            temp.translate(- state.cmd.walk_trasl * total_mult_factor, legs_on_ground_mask).rotate(- state.cmd.walk_rot * total_mult_factor, legs_on_ground_mask)

            # move along step spline if legs not on ground
            legs_stepping_mask = gait_step
            # ahead of time step prediction
            total_mult_factor_step = red * config.STEP_TIME
            target_pose = Pose(config.DEFAULT_POSE, np.ones((6,))).translate(state.cmd.walk_trasl * total_mult_factor_step).rotate(state.cmd.walk_rot * total_mult_factor_step)
            points = [
                self._last_step_pose.copy(),
                self._last_step_pose.copy().translate(np.array([0, 0, config.STEP_HEIGHT])),
                target_pose.copy().translate(np.array([0, 0, config.STEP_HEIGHT])),
                target_pose.copy()
            ]
            temp.bezier(self._gait_step_state, points, legs_stepping_mask)

            # update step state when a substep is completed
            self._gait_step_state += len(gait) / (config.STEP_TIME * config.ENGINE_FPS)
            if self._gait_step_state > 1:
                self._gait_step_state = 0
                self._gait_step = (self._gait_step + 1) % len(gait)
                self._last_step_pose = pose.copy()

            return self, temp
        elif state.cmd.state == 'awake' and state.cmd.mode == 'stand':
            return StandState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        elif state.cmd.state == 'idle':
            return IdleState(state, pose), Pose(config.DEFAULT_POSE, np.ones(shape=6))
        else:
            perr(f'unhandled state in walk state: {state}')


class FiniteStateMachine:
    # current fsm state instance
    _state: typing.Any
    # last commanded pose by the fsm
    _pose: Pose

    def __init__(self, state: RobotState):
        self._state = IdleState(state, state.pose)
        self._pose = state.pose

    def update(self, state: RobotState) -> Pose:
        '''get next state and updated pose'''
        self._state, pose = self._state.update(state, self._pose)
        self._pose = pose.copy()
        # print("current_state:", self._state.identifier)
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

    @staticmethod
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

    def set_hardware_pose(self, pose: Pose):
        '''compute ik and write angles to hardware with current enables'''
        rel_poses = (pose.body_pos - config.POSE_OFFSET) * config.POSE_REL_CONVERT
        angles = np.ravel(np.array([self.relative_ik(rel, leg.dim) for rel, leg in zip(rel_poses, config.legs)]))
        # publish the angles
        self._engine_joint_publisher_msg.position = angles
        # expand leg enable to its 3 joints
        self._engine_joint_publisher_msg.effort = np.ravel(np.tile(pose.enables, (3, 1)).transpose((1, 0)))
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
        msg.body_trasl = np.array(msg.body_trasl)
        msg.body_rot = np.array(msg.body_rot)
        msg.walk_trasl = np.array(msg.walk_trasl)
        msg.walk_rot = np.array(msg.walk_rot)
        self._robot_state.cmd = msg

    def force_sensors_callback(self, msg):
        self._robot_state.force_sensors = np.array(msg.data)

    def update(self):
        self.get_hardware_pose()
        self.set_hardware_pose(self._finite_state_machine.update(self._robot_state))


if __name__ == '__main__':
    rospy.init_node('engine')

    pinfo("starting")

    node = EngineNode()

    rate = rospy.Rate(config.ENGINE_FPS)
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
    pinfo("stopped")
