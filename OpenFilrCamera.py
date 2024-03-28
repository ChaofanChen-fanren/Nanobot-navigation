def openFlirCamera():
    import EasyPySpin
    import cv2
    cap = EasyPySpin.VideoCapture(0)
    if not cap.isOpened():
        print("Camera can't open\nexit")
        return -1

    # cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
    # cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
    return cap


if __name__ == '__main__':
    import cv2

    cap = openFlirCamera()
    while cap.isOpened():
        ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (960, 540))
        # frame = cv2.resize(frame, None, fx=0.25, fy=0.25)
        cv2.imshow("press q to quit", frame)
        key = cv2.waitKey(30)
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
