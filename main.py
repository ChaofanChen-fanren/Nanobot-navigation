import cv2
import time
from Obstacles import Obstacles
from util import openFlirCamera
from common import Robot
import math
from DAQ import DAQ


def Init_DAQ():
    # 初始化参数（你需要根据你的VI填写这些参数的具体值）
    physical_channels = ['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2']  # 物理通道名
    # beta角
    beta = 180
    # alpha角
    alpha = 90
    # 磁场强度B
    B = 20
    # 偏差值 m
    m = -math.pi / 2
    # 频率f
    f = 10
    daq = DAQ(physical_channels)
    daq.set_beta(beta)
    daq.set_alpha(alpha)
    daq.set_f(f)
    daq.set_m(m)
    daq.set_B(B)
    return daq


# 鼠标点击回调函数
def get_mouse_points(event, x, y, flags, param):
    global mouse_x, mouse_y  # 声明全局变量
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Mouse coordinates: (x={x}, y={y})")
        mouse_x, mouse_y = x, y


def imgxy2robotxy(img_height, x, y):
    return x, img_height - y


def update(robot_position, daq, frame):
    m_x, m_y = imgxy2robotxy(frame.shape[0], mouse_x, mouse_y)
    dx = robot_position[0] - m_x
    dy = robot_position[1] - m_y
    if abs(dx) < 2 or abs(dy) < 2:
        daq.stopDaq()
    else:
        # TODO:设置默认不动
        daq.StartDaq()
        b = math.atan(dy / dx)
        beta = 270 - math.degrees(b)
        print(beta)
        daq.set_beta(beta)


# 初始化全局变量
mouse_x, mouse_y = 0, 0
# Open camera. Parameter: camera_index
# cap = cv2.VideoCapture(0)
cap = openFlirCamera()
# Wait for camera init
time.sleep(3)
# 初始化实例
robot = Robot(cap)
# obstacles = Obstacles()
daq = Init_DAQ()
# 初始化鼠标操作
cv2.namedWindow("img")
cv2.setMouseCallback("img", get_mouse_points)
[mouse_x, mouse_y] = robot.get_robot_position()
flag = True
start_time = time.time()
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
    robot.process_frame(frame)
    robot.show_robot_frame(frame)
    # Retrieve and print the current robot position
    # robot_position = robot.get_robot_position()
    # print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")

    # obstacles.find_all_obstacles(frame)
    # # Retrieve and print the current robot position
    # obstacles.show_all_obstacles(frame)

    # Get robot show_frame for visualization
    cv2.imshow("img", frame)
    # cv2.circle(frame, (mouse_x, mouse_y), 5, (255, 0, 0), 5)

    # # TODO:update daq
    # Check if 3 seconds have passed since the last update
    if time.time() - start_time >= 3:
        # Perform the update every 3 seconds
        update(robot.get_robot_position(), daq, frame)
        start_time = time.time()  # Reset the start time

    # daq.set_beta(0)
    if mouse_x != 0 and mouse_y != 0 and flag:
        flag = False
        daq.start()

    # print("mouse: ", mouse_x, ",", mouse_y)
    # 等待按键，如果按下 'q' 键，就退出循环
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# 释放视频捕获设备
cap.release()
# 销毁所有窗口
cv2.destroyAllWindows()
