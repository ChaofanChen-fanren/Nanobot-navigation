from common import Obstacle, Astar
import cv2
import numpy as np


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


def show_ploy(img, ploy):
    mask = np.zeros(img.shape, np.uint8)
    pts = np.array([ploy], np.int32)
    pts = pts.reshape((-1, 1, 2))
    mask = cv2.fillPoly(mask, [pts], (255, 255, 255))
    im = cv2.addWeighted(img, 0.4, mask, 0.6, 0)
    cv2.imshow("img", im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()
obstacle = Obstacle(weights_path="../unet.pth", frame=frame)
# img = cv2.imread("../image/3.png")
inflation_radius = 7  # 障碍物膨胀半径
grid_size = 2.0  # 网格大小
ploy = get_ploy_points(frame)
# ploy 获取为图像坐标系
astar = Astar(obstacle.obstacle_map, inflation_radius, grid_size, ploy=ploy)
sx, sy, gx, gy = get_start_goal(frame)
# print(sx , sy, gx , gy)
rx, ry = astar.planning(*astar.convert_coordinates(sx, sy), *astar.convert_coordinates(gx, gy))
astar.show_animation()
# show_ploy(img, ploy)




#
# img = cv2.imread("../image/3.png")
# param = [img]
# cv2.imshow("select ploy", param[0])
# cv2.setMouseCallback("select ploy", mouse_callback, param=param)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# print(param[1:])