from dataclasses import dataclass
import numpy as np


# =====================
# === ABS CONSTANTS ===
# =====================

RIGHT = 1
LEFT = 0
PI = np.pi
EPSILON = 0.0001


# ===================================
# === NIGHTMARE ENGINE PARAMETERS ===
# ===================================

# ROBOT DIMENSIONS
STAND_MID_LEG_Y = 26.e-2
STAND_OUT_LEG_Y = 20.e-2
STAND_MID_LEG_X = 0.e-2
STAND_OUT_LEG_X = 20.e-2
BODY_LENGTH = 15.5e-2  # X
BODY_MID_WIDTH = 18.6e-2  # Y
BODY_OUT_WIDTH = 13.7e-2  # Y
LEG_COXA_LENGTH = 6.5e-2
LEG_FEMUR_LENGTH = 13.e-2
LEG_TIBIA_LENGTH = 17.e-2

DEFAULT_DIM = np.array([LEG_COXA_LENGTH, LEG_FEMUR_LENGTH, LEG_TIBIA_LENGTH])

NUMBER_OF_LEGS = 6
NUMBER_OF_SERVOS = 19
NUMBER_OF_SERVOS_PER_LEG = 3
NUMBER_OF_SENSORS = 6
NUMBER_OF_SENSORS_PER_LEG = 1

# HARDWARE PARAMS
# used ttys to communicate with servos
TTY_LIST = ["/dev/ttySERVO0", "/dev/ttySERVO1", "/dev/ttySERVO2", "/dev/ttySERVO3"]
# onboard microcontroller communication port
STAT_TTY = "/dev/ttyACM0"
# hardware handler params
STAT_HEADER = [0x55, 0x55, 0x55, 0x55, 0x55]
FORCE_SENSOR_FILTER_VAL = 1e-1
FORCE_SENSOR_CALIBRATION = {18: 38.47, 15: 1.26, 12: 1.63, 9: 3.44, 6: 11.5, 3: 2.45}

# ENGINE PARAMETERS
LEG_KEEPOUT = 4

STAND_HEIGHT = 10.e-2
LEG_ADJ_HEIGHT = 6.e-2
SIT_HEIGHT = 0e-2
ENGINE_FPS = 30
TIME_STAND_UP = 2.5
TIME_SIT = 2.5

STEP_TIME = 3.
STEP_HEIGHT = 14.e-2
MIN_STEP_TIME = 1.
MAX_STEP_LENGTH = 10.e-2

ENGINE_REFERENCE_FRAME = "odom"  # or world
ENGINE_OUTPUT_ODOM_TOPIC = "engine/odom"

# posture
POSTURE_P_GAIN = 0.15

# dynamic terrain adaptation ## STILL IN BETA ##
DYNAMIC_TERRAIN_ADAPTATION_ENABLED = True
MAX_LEG_CORRECTION = 10e-2
# P controller
LEG_P_GAIN = 1e-2
LEG_SETPOINT_FILTER_VAL = 0
# spring-mass
SPRINGINESS = 200
DAMPINESS = 0.05

# walking terrain adaptation
WALKING_TERRAIN_ADAPTATION_ENABLED = False
TOUCHDOWN_TRESHOLD = 0.01
MAX_TOUCHDOWN_DEPTH_SEARCH = 0.1  # 10cm

AUTO_HEIGHT_ADJUSTMENT_ENABLED = False

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

# LEG ADJ SEQUENCES
DOUBLE_SEQUENCES = [[[1, 4], [3, 5], [2, 6]],
                    [[3, 6], [1, 5], [2, 4]]]


# LEG CLASS
@dataclass
class leg_class():
    dim: np.ndarray
    servo_offset: np.ndarray
    side: int
    abs_offset: np.ndarray
    default_pose: np.ndarray
    servo_ang: np.ndarray = np.array([0, 0, 0])


legs = [
    leg_class(  # leg 1
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_LENGTH / 2, -BODY_OUT_WIDTH / 2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, - STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([PI / 4, 0, 0])
    ),
    leg_class(  # leg 2
        dim=DEFAULT_DIM,
        abs_offset=np.array([0, -BODY_MID_WIDTH / 2, 0]),
        default_pose=np.array([STAND_MID_LEG_X, -STAND_MID_LEG_Y, - STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([0, 0, 0])
    ),
    leg_class(  # leg 3
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_LENGTH / 2, -BODY_OUT_WIDTH / 2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, - STAND_HEIGHT]),
        side=RIGHT,
        servo_offset=np.array([-PI / 4, 0, 0])
    ),
    leg_class(  # leg 4
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_LENGTH / 2, BODY_OUT_WIDTH / 2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, STAND_OUT_LEG_Y, - STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([PI / 4, 0, 0])
    ),
    leg_class(  # leg 5
        dim=DEFAULT_DIM,
        abs_offset=np.array([0, BODY_MID_WIDTH / 2, 0]),
        default_pose=np.array([STAND_MID_LEG_X, STAND_MID_LEG_Y, - STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([0, 0, 0])
    ),
    leg_class(  # leg 6
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_LENGTH / 2, BODY_OUT_WIDTH / 2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, STAND_OUT_LEG_Y, - STAND_HEIGHT]),
        side=LEFT,
        servo_offset=np.array([-PI / 4, 0, 0])
    )
]

# POSES
DEFAULT_POSE = np.array([leg.default_pose for leg in legs])
DEFAULT_SIT_POSE = np.array([[pos[0], pos[1], SIT_HEIGHT] for pos in DEFAULT_POSE.copy()])
SERVO_OFFSET = np.append(np.array([leg.servo_offset for leg in legs]).ravel(), 0)
POSE_OFFSET = np.array([leg.abs_offset for leg in legs])
POSE_REL_CONVERT = np.array([[-1 if leg.side else 1, -1 if leg.side else 1, 1] for leg in legs])

# GAITS
GAIT = {'tripod': np.array([[0, 2, 4], [1, 3, 5]]),
        'ripple': np.array([[0, 4], [1, 3], [2, 5]]),
        'wave': np.array([[0], [1], [2], [3], [4], [5]])}

# NAMES
# name of the link at the tip of each leg
LEG_TIPS = ['leg_1_tip', 'leg_2_tip', 'leg_3_tip', 'leg_4_tip', 'leg_5_tip', 'leg_6_tip']
