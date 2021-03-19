# import cv2
import numpy as np


def get_flicker(frame, prev_frame):
    return np.absolute(frame.astype(int) - prev_frame.astype(int)).astype(np.uint8)
