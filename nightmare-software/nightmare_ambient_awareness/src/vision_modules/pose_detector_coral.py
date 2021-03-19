import cv2
import os
import sys

from edgetpu.basic import edgetpu_utils

# import local modules
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)  # noqa

from pose_engine import PoseEngine

model = path + "/models/posenet_mobilenet_v1_075_721_1281_quant_decoder_edgetpu.tflite"

model_width = 640
model_height = 480

devices = edgetpu_utils.ListEdgeTpuPaths(edgetpu_utils.EDGE_TPU_STATE_UNASSIGNED)
engine = PoseEngine(model, devices[0])

EDGES = (
    ('nose', 'left eye'),
    ('nose', 'right eye'),
    ('nose', 'left ear'),
    ('nose', 'right ear'),
    ('left ear', 'left eye'),
    ('right ear', 'right eye'),
    ('left eye', 'right eye'),
    ('left shoulder', 'right shoulder'),
    ('left shoulder', 'left elbow'),
    ('left shoulder', 'left hip'),
    ('right shoulder', 'right elbow'),
    ('right shoulder', 'right hip'),
    ('left elbow', 'left wrist'),
    ('right elbow', 'right wrist'),
    ('left hip', 'right hip'),
    ('left hip', 'left knee'),
    ('right hip', 'right knee'),
    ('left knee', 'left ankle'),
    ('right knee', 'right ankle'),
)


def draw_pose(img, pose, threshold=0.2):
    xys = {}
    for label, keypoint in pose.keypoints.items():
        if keypoint.score < threshold:
            continue
        xys[label] = (int(keypoint.yx[1]), int(keypoint.yx[0]))
        print(keypoint.yx[0])
        img = cv2.circle(img, (int(keypoint.yx[1]), int(keypoint.yx[0])), 5, (0, 255, 0), -1)

    for a, b in EDGES:
        if a not in xys or b not in xys:
            continue
        ax, ay = xys[a]
        bx, by = xys[b]
        img = cv2.line(img, (ax, ay), (bx, by), (0, 255, 255), 2)


def overlay_on_image(frames, result):
    color_image = frames

    # if isinstance(result, type(None)):
    #     return color_image
    img_cp = color_image.copy()

    for pose in result:
        draw_pose(img_cp, pose)

    return img_cp


def detect_poses(frame):
    color_image = cv2.resize(frame, (model_width, model_height))
    prepimg = color_image[:, :, ::-1].copy()

    res = engine.DetectPosesInImage(prepimg)[0]

    if res:
        imdraw = overlay_on_image(color_image, res)
    else:
        imdraw = color_image
    return imdraw
