import EasyPySpin
import numpy as np


def openFlirCamera():
    cap = EasyPySpin.VideoCapture(0)
    if not cap.isOpened():
        print("Camera can't open\nexit")
        return -1

    # cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
    # cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
    return cap


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)