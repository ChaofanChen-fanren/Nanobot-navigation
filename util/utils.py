import EasyPySpin


def openFlirCamera():
    cap = EasyPySpin.VideoCapture(0)
    if not cap.isOpened():
        print("Camera can't open\nexit")
        return -1

    # cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
    # cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
    return cap