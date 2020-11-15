import numpy as np
from dataclasses import dataclass

STAND_HEIGTH = -12.
STAND_MID_LEG_Y = 26.
STAND_OUT_LEG_Y = 20.
STAND_MID_LEG_X = 0.
STAND_OUT_LEG_X = 20.
BODY_LENGTH = 15.5  # X
BODY_MID_WIDTH = 18.6  # Y
BODY_OUT_WIDTH = 13.7  # Y
LEG_COXA_LENGTH = 6.5
LEG_FEMUR_LENGTH = 13.
LEG_TIBIA_LENGTH = 17.
RIGHT = 1
LEFT = 0
PI = np.pi

DEFAULT_DIM = np.array([LEG_COXA_LENGTH, LEG_FEMUR_LENGTH, LEG_TIBIA_LENGTH])


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
        abs_offset=np.array([BODY_LENGTH/2, -BODY_OUT_WIDTH/2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, STAND_HEIGTH]),
        side=RIGHT,
        servo_offset=np.array([-PI/4, 0, 0])
    ),
    leg_class(  # leg 2
        dim=DEFAULT_DIM,
        abs_offset=np.array([0, -BODY_MID_WIDTH/2, 0]),
        default_pose=np.array([STAND_MID_LEG_X, -STAND_MID_LEG_Y, STAND_HEIGTH]),
        side=RIGHT,
        servo_offset=np.array([0, 0, 0])
    ),
    leg_class(  # leg 3
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_LENGTH/2, -BODY_OUT_WIDTH/2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, -STAND_OUT_LEG_Y, STAND_HEIGTH]),
        side=RIGHT,
        servo_offset=np.array([PI/4, 0, 0])
    ),
    leg_class(  # leg 4
        dim=DEFAULT_DIM,
        abs_offset=np.array([-BODY_LENGTH/2, BODY_OUT_WIDTH/2, 0]),
        default_pose=np.array([-STAND_OUT_LEG_X, STAND_OUT_LEG_Y, STAND_HEIGTH]),
        side=LEFT,
        servo_offset=np.array([PI/4, 0, 0])
    ),
    leg_class(  # leg 5
        dim=DEFAULT_DIM,
        abs_offset=np.array([0, BODY_MID_WIDTH/2, 0]),
        default_pose=np.array([STAND_MID_LEG_X, STAND_MID_LEG_Y, STAND_HEIGTH]),
        side=LEFT,
        servo_offset=np.array([0, 0, 0])
    ),
    leg_class(  # leg 6
        dim=DEFAULT_DIM,
        abs_offset=np.array([BODY_LENGTH/2, BODY_OUT_WIDTH/2, 0]),
        default_pose=np.array([STAND_OUT_LEG_X, STAND_OUT_LEG_Y, STAND_HEIGTH]),
        side=LEFT,
        servo_offset=np.array([-PI/4, 0, 0])
    )
]

DEFAULT_POSE = np.array([leg.default_pose for leg in legs])
SERVO_OFFSET = np.append(np.array([leg.servo_offset for leg in legs]).ravel(), 0)
POSE_OFFSET = np.array([leg.abs_offset for leg in legs])
POSE_REL_CONVERT = np.array([[1, -1 if leg.side else 1, 1] for leg in legs])