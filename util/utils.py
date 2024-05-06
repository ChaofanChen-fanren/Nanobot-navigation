import EasyPySpin
import numpy as np
import cv2
from math import factorial
from itertools import product
from scipy.spatial.transform import Rotation as Rot

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
    start_ponint, end_point, sin_point = param[2:4]
    return start_ponint[0], start_ponint[1], end_point[0], end_point[1]



def generate_points(p1, p2, pixel_spacing):
    # 将两个点转换为 numpy 数组
    p1 = np.array(p1)
    p2 = np.array(p2)

    # 计算直线的方向向量
    direction_vector = p2 - p1

    # 计算两点间的距离
    distance = np.linalg.norm(direction_vector)

    # 计算总共需要生成的点数
    num_points = int(distance / pixel_spacing) + 1

    # 计算步长
    step = direction_vector / (num_points - 1)

    # 生成坐标点
    points = [p1]
    for i in range(1, num_points - 1):
        # 计算每个点的坐标
        point = p1 + step * i
        points.append(point)
    points.append(p2)

    return points


def generate_sin_wave(p1, p2, pixel_spacing, frequency=20, amplitude=30):
    # 将两个点转换为 numpy 数组
    p1 = np.array(p1)
    p2 = np.array(p2)

    # 计算直线的方向向量
    direction_vector = p2 - p1

    # 计算两点间的距离
    distance = np.linalg.norm(direction_vector)

    # 计算总共需要生成的点数
    num_points = int(distance / pixel_spacing) + 1

    # 计算步长
    step = direction_vector / (num_points - 1)

    # 生成坐标点
    points = []
    for i in range(num_points):
        # 计算每个点的坐标
        point = p1 + step * i
        x = np.linalg.norm(point - p1)
        y = amplitude * np.sin(2 * np.pi * frequency * x / distance)
        points.append([point[0], point[1] + y])

    return np.array(points)


def generate_circle_path(x0, y0, radius, step_size):
    # 生成圆形路径上的点
    angles = np.linspace(0, 2 * np.pi, int(2 * np.pi * radius / step_size))
    x = x0 + radius * np.cos(angles)
    y = y0 + radius * np.sin(angles) - radius
    return x, y


def generate_square_path(x0, y0, side_length, step_size):
    # 生成正方形路径上的点
    x = []
    y = []
    for i in range(0, side_length + 1, step_size):
        x.append(x0 + i)
        y.append(y0)
    for i in range(0, side_length + 1, step_size):
        x.append(x0 + side_length)
        y.append(y0 + i)
    for i in range(side_length, -1, -step_size):
        x.append(x0 + i)
        y.append(y0 + side_length)
    for i in range(side_length, -1, -step_size):
        x.append(x0)
        y.append(y0 + i)
    return x, y


def generate_sin_path(x0, y0, amplitude, period, step_size):
    # 生成sin函数路径上的点
    x = np.arange(0, period + step_size, step_size)
    y = amplitude * np.sin(2 * np.pi * x / period)
    x += x0
    y += y0
    return x, y


def comb(n, k):
    return factorial(n) // (factorial(k) * factorial(n-k))


def get_bezier_curve(points):
    n = len(points) - 1
    return lambda t: sum(comb(n, i)*t**i * (1-t)**(n-i)*points[i] for i in range(n+1))


def evaluate_bezier(points, total):
    bezier = get_bezier_curve(points)
    new_points = np.array([bezier(t) for t in np.linspace(0, 1, total)])
    return new_points[:, 0], new_points[:, 1]


def points_in_circle(radius):
    for x, y in product(range(int(radius) + 1), repeat=2):
        if x**2 + y**2 <= radius**2:
            yield from set(((x, y), (x, -y), (-x, y), (-x, -y),))


def get_contours(frame):
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (7, 7), 1)
    threshold1, threshold2 = 0, 255
    img_canny = cv2.Canny(img_blur, threshold1, threshold2)
    contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours


def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    # >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]