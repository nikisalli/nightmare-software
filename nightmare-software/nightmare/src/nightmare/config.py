from dataclasses import dataclass
import numpy as np


# =====================
# === ABS CONSTANTS ===
# =====================

RIGHT = 1
LEFT = 0
PI = np.pi
EPSILON = 1e-6


# ===================================
# === NIGHTMARE ENGINE PARAMETERS ===
# ===================================

# ROBOT DIMENSIONS
STAND_MID_LEG_X = 26.e-2
STAND_OUT_LEG_X = 20.e-2
STAND_MID_LEG_Y = 0.e-2
STAND_OUT_LEG_Y = 20.e-2
BODY_LENGTH = 15.5e-2  # X
BODY_MID_WIDTH = 18.6e-2  # Y
BODY_OUT_WIDTH = 13.7e-2  # Y
LEG_COXA_LENGTH = 6.5e-2
LEG_FEMUR_LENGTH = 13.e-2
LEG_TIBIA_LENGTH = 17.e-2

DEFAULT_DIM = np.array([LEG_COXA_LENGTH, LEG_FEMUR_LENGTH, LEG_TIBIA_LENGTH])

NUMBER_OF_LEGS = 6
NUMBER_OF_SERVOS = 18
NUMBER_OF_SERVOS_PER_LEG = 3
NUMBER_OF_SENSORS = 6
NUMBER_OF_SENSORS_PER_LEG = 1

# HARDWARE PARAMS
COMMUNICATION_PORT = "/dev/ttyACM0"
SERVO_PORT = "/dev/ttyACM1"
SENSOR_PORT = "/dev/ttyACM2"

SENSOR_HEADER = [0x55, 0x55, 0x55, 0x55]
# convert raw vals to kg
LOAD_CELLS_CONVERSION_FACTOR = np.array([325334.23661089, 428671.46949402, -448192.25155393, 351489.18146571, -437940.86353348, -303112.40625357])

# ENGINE PARAMETERS
# distance legs must keep from each other
LEG_KEEPOUT_RADIUS = 0.03

STAND_HEIGHT = 10.e-2
LEG_ADJ_HEIGHT = 6.e-2
SIT_HEIGHT = 0.

ENGINE_FPS = 30

TIME_GET_UP_LEG_ADJ = 1.0
TIME_SIT_LEG_ADJ = 1.0
TIME_GET_UP = 2.5
TIME_SIT = 2.5

STEP_TIME = 0.8
STEP_HEIGHT = 6.e-2
MIN_STEP_TIME = 1.
MAX_STEP_LENGTH = 8.e-2

# ROBOT MOVEMENT LIMITS
# walk settings
MAX_WALK_TRASL_VEL = np.array([10e-2, 8e-2, 0])  # m/s
MAX_WALK_ROT_VEL = np.array([0, 0, PI / 10])  # rad/s

MAX_WALK_TRASL_CMD_ACC = np.array([5e-2, 5e-2, 5e-2])  # m/s^2
MAX_WALK_ROT_CMD_ACC = np.array([PI / 10, PI / 10, PI / 10])  # rad/s^2

# body displacement settings
MAX_BODY_TRASL = np.array([9e-2, 9e-2, 12e-2])  # m
MAX_BODY_ROT = np.array([PI / 10, PI / 10, PI / 10])  # rad

MAX_BODY_TRASL_CMD_VEL = np.array([20e-5, 20e-5, 20e-5])  # m/s
MAX_BODY_ROT_CMD_VEL = np.array([PI / 100, PI / 100, PI / 100])  # rad/s

MAX_BODY_TRASL_CMD_ACC = np.array([20e-6, 20e-6, 20e-6])  # m/s^2
MAX_BODY_ROT_CMD_ACC = np.array([PI / 1000, PI / 1000, PI / 1000])  # rad/s^2

# max servo angle
COXA_MAX_ANGLE = PI / 4  # 45 degrees
COXA_MIN_ANGLE = -PI / 4  # -45 degrees
FEMUR_MAX_ANGLE = (2 * PI) / 3  # 120 degrees
FEMUR_MIN_ANGLE = -(2 * PI) / 3  # -120 degrees
TIBIA_MAX_ANGLE = PI / 2  # 90 degrees
TIBIA_MIN_ANGLE = -PI / 2  # -90 degrees

# leg adj sequences
DOUBLE_SEQUENCES = [[[1, 4], [3, 5], [2, 6]],
                    [[3, 6], [1, 5], [2, 4]]]

# URDF PARAMS
URDF_JOINT_OFFSETS = np.array([0.7854, -1.2734, -0.7854, 0, -1.2734, -0.7854, -0.7854, -1.2734, -0.7854, 0.7854, -1.2734, -0.7854, 0, -1.2734, -0.7854, -0.7854, -1.2734, -0.7854])
JOINT_STATE_LABELS = ['leg1coxa', 'leg1femur', 'leg1tibia',
                      'leg2coxa', 'leg2femur', 'leg2tibia',
                      'leg3coxa', 'leg3femur', 'leg3tibia',
                      'leg4coxa', 'leg4femur', 'leg4tibia',
                      'leg5coxa', 'leg5femur', 'leg5tibia',
                      'leg6coxa', 'leg6femur', 'leg6tibia']
URDF_OFFSET_DICT = dict(zip(JOINT_STATE_LABELS, URDF_JOINT_OFFSETS))
LEG_TIPS = ["leg_1_tip", "leg_2_tip", "leg_3_tip", "leg_4_tip", "leg_5_tip", "leg_6_tip"]


# ============================
# === GENERATED PARAMETERS ===
# ============================

# LEG CLASS
@dataclass
class LEG():
    dim: np.ndarray
    servo_offset: np.ndarray
    side: int
    abs_offset: np.ndarray
    default_pose: np.ndarray
    servo_ang: np.ndarray = np.array([0, 0, 0])


legs = [
    LEG(  # leg 1
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_OUT_WIDTH / 2, BODY_LENGTH / 2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, STAND_OUT_LEG_Y, -STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([PI / 4, -0.14874564, 0])
    ),
    LEG(  # leg 2
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_MID_WIDTH / 2, 0, 0]),
        default_pose=np.array([STAND_MID_LEG_X, STAND_MID_LEG_Y, -STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([0, -0.14874564, 0])
    ),
    LEG(  # leg 3
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_OUT_WIDTH / 2, -BODY_LENGTH / 2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, -STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([-PI / 4, -0.14874564, 0])
    ),
    LEG(  # leg 4
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_OUT_WIDTH / 2, -BODY_LENGTH / 2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, -STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([PI / 4, -0.14874564, 0])
    ),
    LEG(  # leg 5
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_MID_WIDTH / 2, 0, 0]),
        default_pose=np.array([-STAND_MID_LEG_X, STAND_MID_LEG_Y, -STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([0, -0.14874564, 0])
    ),
    LEG(  # leg 6
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_OUT_WIDTH / 2, BODY_LENGTH / 2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, STAND_OUT_LEG_Y, -STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([-PI / 4, -0.14874564, 0])
    )
]

DEFAULT_POSE: np.ndarray(shape=(6, 3)) = np.array([leg.default_pose for leg in legs])
DEFAULT_SIT_POSE: np.ndarray(shape=(6, 3)) = np.array([[pos[0], pos[1], SIT_HEIGHT] for pos in DEFAULT_POSE.copy()])
SERVO_OFFSET: np.ndarray(shape=(18,)) = np.array([leg.servo_offset for leg in legs]).ravel()
POSE_OFFSET: np.ndarray(shape=(6, 3)) = np.array([leg.abs_offset for leg in legs])
POSE_REL_CONVERT: np.ndarray(shape=(6, 3)) = np.array([[1 if leg.side else -1, 1 if leg.side else -1, 1] for leg in legs])
SERVO_IDS: np.ndarray(shape=(18,)) = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

# make this poses immutable for safety
DEFAULT_POSE.flags.writeable = False
DEFAULT_SIT_POSE.flags.writeable = False
SERVO_OFFSET.flags.writeable = False
POSE_OFFSET.flags.writeable = False
POSE_REL_CONVERT.flags.writeable = False
SERVO_IDS.flags.writeable = False

# GAITS
GAIT = {'tripod': [np.array([True, False, True, False, True, False]),
                   np.array([False, True, False, True, False, True])],
        'ripple': [np.array([True, False, False, False, True, False]),
                   np.array([False, True, False, True, False, False]),
                   np.array([False, False, True, False, False, True])],
        'wave': [np.array([True, False, False, False, False, False]),
                 np.array([False, True, False, False, False, False]),
                 np.array([False, False, True, False, False, False]),
                 np.array([False, False, False, True, False, False]),
                 np.array([False, False, False, False, True, False]),
                 np.array([False, False, False, False, False, True])]}
