import EasyPySpin
import numpy as np
import cv2


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


def mouse_callback(event, x, y, flags, param):
    # cv.EVENT_LBUTTONDOWN表示鼠标左键向下点击一下
    if event == cv2.EVENT_LBUTTONDOWN:
        img = param[0].copy()
        flag = param[1]
        param.append([x, y])
        cv2.circle(img, (x, y), 10, (0, 255, 0), 2)
        # cv2.putText(param[0], str(x) + "," + str(y), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
        # 将鼠标选的点用直线连起来
        for i in range(2, len(param)):
            cv2.circle(img, param[i], 10, (0, 255, 0), 2)
        if flag:
            for i in range(2, len(param) - 1):
                cv2.line(img, param[i], param[i + 1], (0, 0, 255), 2)
        cv2.imshow("select point", img)
    # -------------------------右键按下清除上一个点-----------------------------
    if event == cv2.EVENT_RBUTTONDOWN:  # 右键点击
        img = param[0].copy()
        flag = param[1]
        if len(param) > 2:
            param.pop()
        for i in range(2, len(param)):
            cv2.circle(img, param[i], 10, (0, 255, 0), 2)
        if flag:
            for i in range(2, len(param) - 1):
                cv2.line(img, param[i], param[i + 1], (0, 0, 255), 2)
        cv2.imshow("select point", img)


def get_ploy_points(frame):
    param = [frame, True]
    cv2.imshow("select point", param[0])
    cv2.setMouseCallback("select point", mouse_callback, param=param)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return param[2:]


def get_start_goal(frame):
    param = [frame, False]
    cv2.imshow("select point", param[0])
    cv2.setMouseCallback("select point", mouse_callback, param=param)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    start_ponint, end_point = param[2:4]
    return start_ponint[0], start_ponint[1], end_point[0], end_point[1]