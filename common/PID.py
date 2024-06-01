import math
from .Astar import Astar
from .Obstacle import Obstacle
from util import get_ploy_points, get_start_goal, generate_points, generate_sin_wave, generate_circle_path, generate_square_path, generate_sin_path
from .Thrombus import Thrombus
from util import openFlirCamera
import cv2
class PID:
    def __init__(self, frame,
                 x0, y0, contours,
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
        self.setValue_x = self.x0
        self.setValue_y = self.y0
        self.lastErr_x = 0
        self.lastErr_y = 0
        self.errSum_x = 0
        self.errSum_y = 0
        self.errSumLimit_x = 1000
        self.errSumLimit_y = 1000

        self.last_Err_dm = 0
        self.index_thrombus_path = None

        # 通过Astar算法计算路径
        self.path_x_list, self.path_y_list = self.get_path(frame, contours)
        # 获取给定路线
        # self.path_x_list, self.path_y_list = self.get_a_path()
        print(f"----len{len(self.path_x_list)}")
        self.index_path = 0
        self.is_arrived_dis = 15.0

        # 定义到达血栓区域的index路径
        self.is_rotate = False
        self.rotate_index = 0

    @staticmethod
    def imgxy2robotxy(img_height, x, y):
        return x, img_height - y


    def get_a_path(self):
        # 设置起始点和幅度、周期
        x0, y0 = self.x0, self.y0
        ################### Sin ############################
        amplitude = 210
        period = 400
        # 设置步长
        step_size = 15
        # 生成sin函数路径
        x, y = generate_sin_path(x0, y0, amplitude, period, step_size)
        # ################## Square ########################
        # side_length = 210
        # # 设置步长
        # step_size = 30
        # # 生成正方形路径
        # x, y = generate_square_path(x0, y0, side_length, step_size)
        ################## Circle ########################
        # radius = 150
        # # 设置步长
        # step_size = 30
        # # 生成圆形路径
        # x, y = generate_circle_path(x0, y0, radius, step_size)

        return x, y


    def get_path(self, frame, contours=None):
        # obstacle = Obstacle(weights_path="../unet.pth", frame=frame)
        obstacle = Obstacle(weights_path="./unet.pth", frame=frame)
        inflation_radius = 3  # 障碍物膨胀半径
        robot_radius = 5  # 机器人的半径
        grid_size = 4.0  # 网格大小
        ploy = get_ploy_points(frame)
        # ploy 获取为图像坐标系
        astar = Astar(obstacle.obstacle_map, inflation_radius, robot_radius, grid_size, ploy=ploy)
        sx, sy, gx, gy = get_start_goal(frame)
        rx, ry = astar.planning(*astar.convert_coordinates(sx, sy), *astar.convert_coordinates(gx, gy))
        # 数组坐标系 转换为 机器人坐标系  数组-》图像-》机器人  路径是倒推
        rx, ry = rx[::-1], ry[::-1]
        #
        path_x_list, path_y_list = [], []
        # gen_points = generate_points((self.x0, self.y0), self.imgxy2robotxy(img_height=frame.shape[0], x=gx, y=gy),  4)
        gen_points = generate_points((self.x0, self.y0), self.imgxy2robotxy(img_height=frame.shape[0], x=ry[0], y=rx[0]),  4)

        # print(frame.shape[0])
        for point in gen_points:
            path_x_list.append(point[0])
            path_y_list.append(point[1])

        # print("-------")

        for kx, ky in zip(rx, ry):
            ix, iy = self.imgxy2robotxy(img_height=frame.shape[0], x=ky, y=kx)
            path_x_list.append(ix)
            path_y_list.append(iy)
        path_x_list, path_y_list = path_x_list[::5], path_y_list[::5]

        self.index_thrombus_path = len(path_x_list)
        # 添加通血栓(thrombus)路径
        # cap = openFlirCamera()
        # ret, frame = cap.read()
        # cap.release()
        # frame = cv2.resize(frame, (1080, 1080))
        # if contours is not None:
        #     cv2.drawContours(frame, contours, -1, (0, 0, 0), cv2.FILLED)
        # thrombus = Thrombus(weights_path="./best_thrombus_model.pth", frame=frame)
        # px, py = thrombus.get_px_py()
        # for i in range(10):
        #     for kx, ky in zip(px, py):
        #         ix, iy = self.imgxy2robotxy(img_height=frame.shape[0], x=kx, y=ky)
        #         path_x_list.append(ix)
        #         path_y_list.append(iy)

        return path_x_list, path_y_list

    def GetCalcuValue(self):
        # x = self.x0 + t
        # y = self.y0 + 200*math.sin(t/50)
        # x = self.path_x_list[t]
        # y = self.path_y_list[t]
        self.index_path += 1
        # if self.index_path < len(self.path_x_list):
        x = self.path_x_list[self.index_path]
        y = self.path_y_list[self.index_path]
        return x, y

    def clear_pid(self):
        self.last_Err_dm = 0
        # self.lastErr_y = 0
        # self.errSum_x = 0
        # self.errSum_y = 0

    # 限幅函数
    def limitIntegralTerm(self, term, limit):
        if term > limit:
            return limit
        elif term < -limit:
            return -limit
        else:
            return term

    def is_arrived(self, robot_position):
        dis = math.dist(robot_position, (self.setValue_x, self.setValue_y))
        return True if dis < self.is_arrived_dis else False

    def update_rotate_index(self):
        self.rotate_index += 1
        if self.rotate_index == 10:
            self.is_rotate = False
            self.rotate_index = 0

    # 位置式PID
    def pidPosition(self, robot_position):
        print(f"X:{robot_position[0]} , Y: {robot_position[1]}")
        # self.setValue_x, self.setValue_y = self.GetCalcuValue(t)
        # 到达当前设定目标点后，更新下一个目标点，清楚pid 参数 Err=0 errSum=0

        if self.index_path < self.index_thrombus_path:  # 行驶到血栓区域路径
            if self.is_arrived(robot_position):
                self.setValue_x, self.setValue_y = self.GetCalcuValue()
                self.clear_pid()
        elif self.index_thrombus_path <= self.index_path < len(self.path_x_list):  # 行驶血栓路径
            if self.is_rotate:  # 当前状态是否要旋转
                self.updata_rotate_index()
                B, f, beta, alpha = 5, 70, 0, 0
                return B, f, alpha, beta
            else:
                if self.is_arrived(robot_position):
                    self.setValue_x, self.setValue_y = self.GetCalcuValue()
                    self.is_rotate = True  # 到达目标点进行自旋十秒
        else:
            print("整个任务完成啦啦啦啦！！！！！！！！！！！！！！！！！！！！！！！！")
            return 0, 0, 0, 90  # 返回 beta=0， B=0

        # 到达血栓目标点判断距离变小
        if self.index_path == self.index_thrombus_path:
            self.is_arrived_dis = 5.0

        # PD 控制器
        # P
        d_x = self.setValue_x - robot_position[0]
        d_y = self.setValue_y - robot_position[1]
        # d_x = robot_position[0] - self.setValue_x
        # d_y = robot_position[1] - self.setValue_y
        # dm
        dm = math.sqrt(d_x * d_x + d_y * d_y)
        d_err_dm = dm - self.last_Err_dm
        self.last_Err_dm = dm
        kp_b, kd_b, kp_f, kd_f = 0.08, 0.001, 0.65, 0.10
        # kp_b, kd_b, kp_f, kd_f = 0.08, 0.001, 1.30, 0.10
        # B = kp_b*dm + kd_b*d_err_dm
        B = 5
        f = kp_f * dm + kd_f * d_err_dm
        f = int(f)
        if f > 70:
            f = 70
        beta = self.cal_beta(dx=d_x, dy=d_y)
        print(f"-----PID UPDATE f:{f}, B:{B}, dm:{dm}, beta:{beta}")
        alpha = 90
        return B, f, alpha, beta

    def plot_list(self):
        position_list = list()
        for index in range(len(self.path_x_list)):
            x, y = self.path_x_list[index], self.path_y_list[index]
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
