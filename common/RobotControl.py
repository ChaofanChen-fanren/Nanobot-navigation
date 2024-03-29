import numpy as np
import matplotlib.pyplot as plt
import math
from .Astar import Astar
from util import distance


"""
定义状态类，用于Robot过程中每一个时刻的状态
    time：状态对应的时刻
    x、y：Robot的坐标
    velocity：Robot的速度
    yaw：偏航角，仿真只在二维空间，所以只定义一个角
    yaw_rate：角速度
    yaw_acceleration：角加速度，代码中没有用到，在这里定义只是方便更加靠近真实的船
    cost：每走一步对应的成本
"""


class State(object):
    def __init__(self, current_time, x, y, velocity, yaw, yaw_rate, yaw_acceleration, cost):
        self.time = current_time
        self.x = x
        self.y = y
        self.velocity = velocity
        self.yaw = yaw
        self.yaw_rate = yaw_rate
        self.yaw_acceleration = yaw_acceleration
        self.cost = cost


"""
定义航行过程中存储Robot所有状态和利用船的状态进行DWA的类-------
states：列表，列表的元素是上面定义的state类，用于存储航行过程中的所有时刻的状态
obstacle_map： 二维障碍物地图，0表示无障碍物，1表示有障碍物
obstacle：列表，元素是障碍点的坐标，这里的障碍点可以是建图完成之后再传入，或者通过传感器探测实时更新
obstacle_windows：本意是定义一个船探测障碍物的探测半径，避障只需要考虑船能探测到的范围内进行就可以
运动参数：
    dt：时间间隔
    max_velocity： 最大线速度
    min_velocity： 最小线速度
    max_linear_acc：最大线性加速度
    max_yaw_rate：最大角速度
    velocity_resolution：线速度搜索间隔
    yaw_rate_resolution：角速度搜索间隔
    predict_time：向前预测时间
    goal_cost_coefficient：到目标点的成本系数
    velocity_cost_coefficient：速度的成本系数
    safe_radius：安全半径
"""


class RobotControl(object):
    def __init__(self, initial_state, obstacle, goal):
        # 初始状态
        self.states = [initial_state]  # 存储航行过程中的所有状态
        self.obstacle_map = obstacle.obstacle_map  # 二维障碍物地图
        self.obstacle_points = obstacle.get_obstacle_points()  # 记录所有障碍物的坐标点

        # self.goal = goal  # 目标点
        self.grid_size = 2.0  # 网格大小
        self.ploy = [[40, 253], [297, 472], [531, 229], [261, 7]]
        self.arrive = False  # 是否到达目标点
        self.obstacle_windows = np.array([10, 10])  # Robot所能探测的障碍物半径

        # Robot的运动参数 robot parameter
        self.max_velocity = 10.0  # 最大线速度 [m/s] # 30
        self.min_velocity = -10.0  # 最小线速度 [m/s]
        self.max_yaw_rate = 180.0 * math.pi / 180.0  # 最大角速度 [rad/s]
        self.max_linear_acc = 10  # 最大线性加速度 [m/ss] # 20
        self.max_yaw_acc = 720.0 * math.pi / 180  # 最大角速度 [rad/ss]
        self.velocity_resolution = 0.5  # 线速度搜索间隔 [m/s] # 1
        self.yaw_rate_resolution = 10 * math.pi / 180  # 角速度搜索间隔 [rad/s]
        self.dt = 0.1  # 时间间隔 [s] Time tick for motion prediction
        self.predict_time = 3.0  # 向前预测时间
        self.angle_cost_coefficient = 10  # 偏差角度的成本系数
        self.goal_cost_coefficient = 20  # 到目标点的成本系数
        self.velocity_cost_coefficient = 5.0  # 速度的成本系数
        self.obstacle_cost_coefficient = 1.0  # 障碍物的成本系数
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        # robot为矩形机器人 rectangle
        self.robot_radius = 1.0
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        self.inflation_radius = 5  # 障碍物膨胀半径
        # self.safe_radius = 3  # 安全半径

        self.goal_list = self.get_goal_list(self.states[-1].x, self.states[-1].y, goal[0], goal[1])  # 获取所有的目标点
        self.goal_index = 0
        self.goal = self.goal_list[self.goal_index]

    # 通过Astar算法求出全局路径
    def get_goal_list(self, sx, sy, gx, gy):
        astar = Astar(self.obstacle_map, self.inflation_radius, self.grid_size, ploy=self.ploy)
        rx, ry = astar.planning(sx, sy, gx, gy)
        # 获取的点是倒着的，需要翻转
        return list(zip(rx, ry))[::-1]

    # 更新下一个目标
    def update_goal(self):
        self.goal_index += 1
        if self.goal_index < len(self.goal_list):
            self.goal = self.goal_list[self.goal_index]
            return True
        return False

    # 运动函数，根据上面说的运动公式
    # yaw(k + 1) = yaw(k) + yaw_rate * dt
    # x(k + 1) = x(k) + v*cos(yaw)*dt
    # y(k + 1) = y(k) + v*sin(yaw)*dt
    def motion(self, velocity, yaw_rate, is_add=False):
        temp_state = State(self.states[-1].time + self.dt,  # 当前时刻
                           self.states[-1].x + velocity * math.cos(self.states[-1].yaw) * self.dt,  # x坐标点
                           self.states[-1].y + velocity * math.sin(self.states[-1].yaw) * self.dt,  # y坐标点
                           velocity,  # 当前速度
                           self.states[-1].yaw + yaw_rate * self.dt,  # 当前偏航角
                           yaw_rate,  # 当前角速度
                           (yaw_rate - self.states[-1].yaw_rate) / self.dt,  # 当前角加速度
                           0)  # 所消耗的损失
        if is_add:
            self.states.append(temp_state)
        return temp_state

    # ------------------------------------
    # 动态窗口定义
    # ------------------------------------
    def motion_windows(self):
        current_velocity = self.states[-1].velocity
        current_yaw_rate = self.states[-1].yaw_rate
        current_max_velocity = np.min([self.max_velocity, current_velocity + self.max_linear_acc * self.dt])
        current_min_velocity = np.max([self.min_velocity, current_velocity - self.max_linear_acc * self.dt])
        current_max_yaw_rate = np.min([self.max_yaw_rate, current_yaw_rate + self.max_yaw_acc * self.dt])
        current_min_yaw_rate = np.max([-self.max_yaw_rate, current_yaw_rate - self.max_yaw_acc * self.dt])

        return current_min_velocity, current_max_velocity, current_min_yaw_rate, current_max_yaw_rate

    # ------------------------------------
    # 四项成本函数的定义 方位角偏差成本、目标成本、速度成本、障碍物碰撞成本
    # ------------------------------------
    def cost_angle(self, trajectory):
        dx = self.goal[0] - trajectory[-1].x
        dy = self.goal[1] - trajectory[-1].y
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1].yaw
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost * self.angle_cost_coefficient

    def cost_goal(self, trajectory):
        dx = self.goal[0] - trajectory[-1].x
        dy = self.goal[1] - trajectory[-1].y
        return math.hypot(dx, dy)*self.goal_cost_coefficient

    def cost_velocity(self, trajectory):
        return (self.max_velocity - trajectory[-1].velocity) * self.velocity_cost_coefficient

    def get_obstacle_windows(self, points):
        s = set()
        for i in points:
            for j in range(-self.obstacle_windows[0], self.obstacle_windows[0]):
                for k in range(-self.obstacle_windows[1], self.obstacle_windows[1]):
                    ox, oy = int(i[0] - j), int(i[1] - k)
                    if (0 <= ox < self.obstacle_map.shape[0]
                            and 0 <= oy < self.obstacle_map.shape[1]
                            and self.obstacle_map[ox][oy] == 1):
                        s.add((ox, oy))

        return np.array(list(s))

    def cost_obstacle(self, locus):
        # dis = []
        # for i in locus:
        #     for j in range(self.obstacle_windows[0]):
        #         for k in range(self.obstacle_windows[1]):
        #             ox, oy = int(i.x - j), int(i.y - k)
        #             if (ox < self.obstacle_map.shape[0] and oy < self.obstacle_map.shape[1]
        #                     and self.obstacle_map[ox][oy] == 1):
        #                 dis.append(distance(np.array([i.x, i.y]), np.array([ox, oy])))
        #     # for ii in self.obstacle:
        #     #     dis.append(distance(np.array([i.x, i.y]), ii))
        # if len(dis) == 0:
        #     dis.append(1e8)
        # dis_np = np.array(dis)
        # return 1.0 / (np.min(dis_np) + 1e-8)
        trajectory_list = []
        for t in locus:
            x = [t.x, t.y, t.yaw, t.velocity, t.yaw_rate]
            trajectory_list.append(x)
        trajectory = np.array(trajectory_list)

        obstacle_points = self.get_obstacle_windows(trajectory[:, 0:2])
        obstacle_points = np.array(obstacle_points)
        if len(obstacle_points) == 0:
            return (1.0 / 1e8) * self.obstacle_cost_coefficient
        else:
            ox = obstacle_points[:, 0]
            oy = obstacle_points[:, 1]
            dx = trajectory[:, 0] - ox[:, None]
            dy = trajectory[:, 1] - oy[:, None]
            r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        np.transpose(rot, [2, 0, 1])
        local_ob = obstacle_points[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2

        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return np.inf
        min_r = np.min(r)
        return (1.0 / min_r) * self.obstacle_cost_coefficient  # OK

    def cost_total(self, locus):
        return self.cost_goal(locus) + self.cost_velocity(locus) + self.cost_obstacle(locus) + self.cost_angle(locus)

    # -----------------------------------
    # 遍历动态窗口内所有轨迹，调用成本函数计算成本并且择优计算最优速度与角速度
    # -----------------------------------
    def search_for_best_uv(self):
        current_min_velocity, current_max_velocity, current_min_yaw_rate, current_max_yaw_rate = self.motion_windows()
        best_vy = np.array([0, 0])
        best_cost = np.inf
        # original_states = self.states[:]  # self.motion会更新self.states[:]
        best_states = []  # 记录搜索最好路线

        for velocity in np.arange(current_min_velocity, current_max_velocity, self.velocity_resolution):
            for yaw in np.arange(current_min_yaw_rate, current_max_yaw_rate, self.yaw_rate_resolution):
                searched_states = [self.states[-1]]  # 存储在predict_time时间内所有间隔dt的所有的Robot状态
                # for t in np.arange(0,self.predict_time, self.dt):
                #     locus.append(self.motion(i,ii))
                t = 0
                while t <= self.predict_time:
                    searched_states.append(self.motion(velocity, yaw))
                    t = t + self.dt
                temp_cost = self.cost_total(searched_states)  # 当前速度和角速度下路线的所消耗的成本
                if best_cost >= temp_cost:
                    best_cost = temp_cost
                    best_vy = [velocity, yaw]
                    # best_locus=copy.deepcopy(locus)
                    best_states = searched_states
                    if abs(velocity) < self.robot_stuck_flag_cons \
                            and abs(self.states[-1].velocity) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_vy[1] = -self.max_yaw_acc
        self.show_animation(best_states)
        return best_vy, best_cost

    # -----------------------------------------------------------------
    # 用matplotlib画图，一帧一帧图像交替，展示动态效果 显示时候为图像坐标系
    # x, y坐标要互换
    # -----------------------------------------------------------------
    def show_animation(self, show_states):
        plt.cla()
        # 显示图像
        plt.imshow(self.obstacle_map, cmap='gray')
        # plt.scatter(self.obstacle[:, 0], self.obstacle[:, 1], s=5)
        # 显示目标点
        plt.plot([point[1] for point in self.goal_list], [point[0] for point in self.goal_list], "g-")
        plt.plot(self.goal[1], self.goal[0], "ro")
        # 显示最佳的搜索路线
        x = []
        y = []
        for state in show_states:
            x.append(state.x)
            y.append(state.y)
        plt.plot(y, x, "g-")
        # 显示当前Robot的坐标
        plt.plot(self.states[-1].y, self.states[-1].x, "bo")
        # 显示当前机器人的朝向（用箭头显示）
        plt.arrow(self.states[-1].y, self.states[-1].x, 2 * math.sin(self.states[-1].yaw),
                  2 * math.cos(self.states[-1].yaw),
                  head_length=30, head_width=10)
        # plt.grid(True)
        # plt.xlim([-10, self.goal[0] * 1.3])
        # plt.ylim([-10, self.goal[1] * 1.3])
        # plt.xlim(0, self.map.shape[1])
        # plt.ylim(self.map.shape[0], 0)  # 翻转y轴，以匹配图像的坐标系
        plt.pause(0.001)

    # -------------------------------------------------------------------
    # 安全检测，判断船离障碍物是不是撞上了 检测是否发生了碰撞
    # -------------------------------------------------------------------
    def check_collision(self):
        position = np.array([[self.states[-1].x, self.states[-1].y]])
        obstacle_points = self.get_obstacle_windows(position)

        for obstacle_point in obstacle_points:
            if distance(position[0], obstacle_point) <= self.robot_radius:
                print("撞上了！")
                return True

        return False
