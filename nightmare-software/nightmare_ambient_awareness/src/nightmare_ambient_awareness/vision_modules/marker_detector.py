import cv2
import numpy as np

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


matrix_coefficients = cv2.UMat(np.array([[1.58881897e3, 0.00000000e0, 3.32894169e2],
                                         [0.00000000e0, 1.96180421e3, 2.30963773e2],
                                         [0.00000000e0, 0.00000000e0, 1.00000000e0]], dtype=np.float32))

distortion_coefficients = cv2.UMat(np.array([6.51962965e1, 2.51962589e1, -4.30591582e3, -3.57087185e2, -9.54851549e2], dtype=np.float32))


def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    parameters = cv2.aruco.DetectorParameters_create()  # Marker detection parameters
    return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


def overlay_marker_on_image(frame, res):
    corners, ids, _ = res
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    if np.all(ids is not None):
        for i, _ in enumerate(ids):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            (rvec - tvec).any()
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
    return frame
