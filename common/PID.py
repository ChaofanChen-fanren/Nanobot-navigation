import math
from .Astar import Astar
from .Obstacle import Obstacle
from util import get_ploy_points, get_start_goal, generate_points, generate_sin_wave
import cv2


class PID:
    def __init__(self, frame,
                 x0, y0,
                 p1=0.3, i1=0.001, d1=0.001,
                 p2=0.3, i2=0.001, d2=0.001,
                 f=13):
        # 设置PID参数
        self.k_p1 = p1
        self.k_p2 = p2
        self.k_i1 = i1
        self.k_i2 = i2
        self.k_d1 = d1
        self.k_d2 = d2

        # 机器人的初始坐标
        self.x0 = x0
        self.y0 = y0

        # 设置频率
        self.f = f

        self.uPrevious = 0
        self.uCurent = 0
        self.setValue_x = 0
        self.setValue_y = 0
        self.lastErr_x = 0
        self.lastErr_y = 0
        self.errSum_x = 0
        self.errSum_y = 0
        self.errSumLimit_x = 1000
        self.errSumLimit_y = 1000

        self.last_Err_dm = 0

        # 通过Astar算法计算路径
        self.path_x_list, self.path_y_list = self.get_path(frame)

    @staticmethod
    def imgxy2robotxy(img_height, x, y):
        return x, img_height - y

    def get_path(self, frame):
        # obstacle = Obstacle(weights_path="../unet.pth", frame=frame)
        obstacle = Obstacle(weights_path="./unet.pth", frame=frame)
        inflation_radius = 10  # 障碍物膨胀半径
        grid_size = 5.0  # 网格大小
        ploy = get_ploy_points(frame)
        # ploy 获取为图像坐标系
        astar = Astar(obstacle.obstacle_map, inflation_radius, grid_size, ploy=ploy)
        sx, sy, gx, gy, sin_x, sin_y = get_start_goal(frame)
        rx, ry = astar.planning(*astar.convert_coordinates(sx, sy), *astar.convert_coordinates(gx, gy))
        # 数组坐标系 转换为 机器人坐标系  数组-》图像-》机器人  路径是倒推
        rx, ry = rx[::-1], ry[::-1]
        path_x_list, path_y_list = [], []
        gen_points = generate_points((self.x0, self.y0), self.imgxy2robotxy(img_height=frame.shape[0], x=ry[0], y=rx[0]),  5)

        for point in gen_points:
            path_x_list.append(point[0])
            path_y_list.append(point[1])

        for kx, ky in zip(rx, ry):
            ix, iy = self.imgxy2robotxy(img_height=frame.shape[0], x=ky, y=kx)
            path_x_list.append(ix)
            path_y_list.append(iy)
        sin_points = generate_sin_wave(self.imgxy2robotxy(frame.shape[0], x=ry[-1], y=rx[-1]),
                                       self.imgxy2robotxy(frame.shape[0], x=sin_x, y=sin_y), pixel_spacing=10)
        for point in sin_points:
            path_x_list.append(point[0])
            path_y_list.append(point[1])
        return path_x_list[::2], path_y_list[::2]

    def GetCalcuValue(self, t):
        # x = self.x0 + t
        # y = self.y0 + 200*math.sin(t/50)
        x = self.path_x_list[t]
        y = self.path_y_list[t]
        return x, y

    # 限幅函数
    def limitIntegralTerm(self, term, limit):
        if term > limit:
            return limit
        elif term < -limit:
            return -limit
        else:
            return term

    # 位置式PID
    def pidPosition(self, robot_position, t):
        print(f"X:{robot_position[0]} , Y: {robot_position[1]} , T:{t:.2f}\n")
        self.setValue_x, self.setValue_y = self.GetCalcuValue(t)
        # P
        d_x = self.setValue_x - robot_position[0]
        d_y = self.setValue_y - robot_position[1]
        # d_x = robot_position[0] - self.setValue_x
        # d_y = robot_position[1] - self.setValue_y
        # dm
        dm = math.sqrt(d_x * d_x + d_y * d_y)
        d_err_dm = dm - self.last_Err_dm
        self.last_Err_dm = dm

        kp_b, kd_b, kp_f, kd_f = 0.08, 0.001, 0.42, 0.10

        B = kp_b*dm + kd_b*d_err_dm
        B = 2
        if B > 15:
            B = 15
        f = kp_f*dm + kd_f*d_err_dm
        if f > 70:
            f = 70
        print(f"-----PID UPDATE f:{f}, B:{B}, dm:{dm}")
        beta = self.cal_beta(dx=d_x, dy=d_y)
        # f = 10
        return beta, B, f

    def plot_list(self):
        position_list = list()
        for t in range(len(self.path_x_list)):
            x, y = self.GetCalcuValue(t)
            position_list.append((int(x), int(y)))
        return position_list

    def cal_beta(self, dx, dy):
        if dx != 0:
            alpha = math.atan(dy / dx)
            beta = int(270 - math.degrees(alpha)) if dx > 0 else int(90 - math.degrees(alpha))
        else:
            beta = 0 if dy < 0 else 180
        return beta

    def cal_B(self, dx, dy, t):
        # B = math.sqrt(dx * dx + dy * dy) / math.cos(2 * math.pi * self.f * t)
        B = math.sqrt(dx * dx + dy * dy)
        if B > 25:
            B = 25
        return B

    def cal_f(self, dx, dy, t):
        f = 1.0*(math.fabs(dx))

    def set_f(self, value):
        self.f = value

    # 增量式PID
    def pidIncrease(self, curValue):
        self.uCurent = self.pidPosition(curValue)
        outPID = self.uCurent - self.uPrevious
        self.uPrevious = self.uCurent
        return outPID
