# import pytest
from common import Robot, Obstacle, DAQ, Astar
import matplotlib.pyplot as plt
import cv2
from util import openFlirCamera
import numpy as np
import time
import math


def test_open_filr_camera():
    cap = openFlirCamera()
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (960, 540))
        # frame = cv2.resize(frame, None, fx=0.25, fy=0.25)
        cv2.imshow("press q to quit", frame)
        key = cv2.waitKey(30)
        if key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def test_one_robot():
    # Open camera. Parameter: camera_index
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)
    # Parameter: camera_index frame_width frame_height
    frame_width, frame_height = 640, 480
    robot = Robot(cap=cap, frame_width=frame_width, frame_height=frame_height)
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (frame_width, frame_height))
        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            cv2.waitKey(10)
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


def test_multiple_robot():
    # Open camera. Parameter: camera_index frame_width frame_height
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)

    # Parameter: camera_index
    frame_width, frame_height = 640, 480
    robot = Robot(cap, frame_width=frame_width, frame_height=frame_height)
    robot1 = Robot(cap, frame_width=frame_width, frame_height=frame_height)

    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (frame_width, frame_height))
        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            robot1.process_frame(frame)
            robot1.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            robot_position1 = robot1.get_robot_position()
            print(f"Robot Position: {robot_position1[0]}  {robot_position1[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            key = cv2.waitKey(10)
            if key == ord("q"):
                break
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


def test_daq_start():
    # 初始化参数（你需要根据你的VI填写这些参数的具体值）
    physical_channels = ['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2']  # 物理通道名
    # beta角
    beta = 0
    # alpha角
    alpha = 90
    # 磁场强度B
    B = 20
    # 偏差值 m
    m = -math.pi/2
    # 频率f
    f = 10
    Daq = DAQ(physical_channels)
    Daq.set_beta(beta)
    Daq.set_alpha(alpha)
    Daq.set_f(f)
    Daq.set_m(m)
    Daq.set_B(B)
    Daq.start()


def test_unet_obstacle():
    ob = Obstacle(weights_path="../unet.pth", image_path='../image/1.jpg')
    # 测试get_obstacle_points
    # ox, oy = ob.get_obstacle_points()
    # print(ox, oy)

    plt.figure()
    plt.subplot(2, 2, 1)
    # 将PIL图像转换为NumPy数组
    array = np.array(ob.get_current_obstacle_map())
    # 使用matplotlib显示图像
    plt.imshow(array, cmap='gray')  # 使用灰度色图来表示白色和黑色

    plt.subplot(2, 2, 2)
    plt.imshow(ob.show_viz_img())

    # 测试update函数
    ob.update("../image/22 copy.jpg")
    plt.subplot(2, 2, 3)
    # 将PIL图像转换为NumPy数组
    array = np.array(ob.get_current_obstacle_map())
    # 使用matplotlib显示图像
    plt.imshow(array, cmap='gray')  # 使用灰度色图来表示白色和黑色

    plt.subplot(2, 2, 4)
    plt.imshow(ob.show_viz_img())

    plt.show()


def test_astar_path():
    ob = Obstacle(weights_path="../unet.pth", image_path='../image/1.jpg')
    obstacle_map = ob.obstacle_map
    # 设置起点和终点
    sx = 379
    sy = 126
    gx = 190
    gy = 350
    # (图像坐标系)
    # sx = 249
    # sy = 265
    # gx = 390
    # gy = 110
    # (图像坐标系)
    # sx = 467
    # sy = 88
    # gx = 498
    # gy = 88

    # 网格大小
    grid_size = 2.0
    # 机器人半径
    robot_radius = 5.0
    # 实例化，传入障碍物，网格大小
    ploy = [[40, 253], [297, 472], [531, 229], [261, 7]]
    astar = Astar(obstacle_map, robot_radius, grid_size, ploy=ploy)
    import time
    time_begin = time.time()
    # 求解路径，返回路径的 x 坐标和 y 坐标列表
    rx, ry = astar.planning(*astar.convert_coordinates(sx, sy), *astar.convert_coordinates(gx, gy))
    # simpler_points = [(x, y) for x, y in zip(rx, ry)]
    # simpler_points = simplify_path(simpler_points)
    # print(len(rx), len(simpler_points))
    # print(rx)
    # print([point[0] for point in simpler_points])
    time_end = time.time()
    print(f"路径规划消耗时间：{time_end - time_begin}秒")
    astar.show_animation()


if __name__ == '__main__':
    # test_open_filr_camera()
    # test_one_robot()
    # test_multiple_robot()
    # test_unet_obstacle()
    # test_daq_start()
    test_astar_path()
