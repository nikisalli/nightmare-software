import cv2
import numpy as np


def get_static_saliency(frame):
    # image = cv2.resize(f, (640, 480))
    saliency = cv2.saliency.StaticSaliencySpectralResidual_create()
    saliencyMap = saliency.computeSaliency(frame)[1]
    saliencyMap = (saliencyMap * 255).astype("uint8")
    return saliencyMap


def get_intensity(frame):
    return frame.mean(axis=2).astype(np.uint8)
