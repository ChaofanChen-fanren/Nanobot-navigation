import math
import matplotlib.pyplot as plt

"""
Astar类
    original_obstacle_map：存储原始的障碍物地图
    obstacle_map:存储缩小resolution倍的障碍物
    robot_radius：机器人安全半径
    motion：存储机器人的动作和所消耗的损失[dx,dy,cost]
Astar类使用坐标系：数组的坐标系


数组坐标系->图像坐标系：self.convert_Coordinates
图像坐标系->数组坐标系：self.convert_Coordinates
注意如果绘图的坐标系不是和数组的坐标系不一样
"""


class Astar:
    def __init__(self, obstacle_map, inflation_radius, resolution, ploy):
        self.original_obstacle_map = obstacle_map
        self.inflation_radius = inflation_radius
        # 膨胀障碍物，如果障碍物和网格之间的距离小，机器人无法通行，对障碍物膨胀
        # TODO:根据机器人的半径设置膨胀障碍物
        self.original_inflate_obstacle_map = self.inflate_obstacles(self.original_obstacle_map)

        # 设置原始障碍物map的x轴y轴边界 calc_obstacle_map函数中初始化
        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        # 设置所框选的区域 使用 lambda 函数交换每个元组的元素位置变换坐标系到数组坐标系
        self.ploy = list(map(lambda xy: (xy[1], xy[0]), ploy))

        # 设置分辨率，即缩放的倍数
        self.resolution = resolution
        # 地图的x和y方向的栅格个数
        self.x_width = None
        self.y_width = None

        # 解析original_obstacle_map
        self.obstacle_map = self.calc_obstacle_map()
        self.motion = self.get_motion_model()

        # 记录所有的搜索过的点(数组坐标系)
        self.sx, self.sy = None, None
        self.gx, self.gy = None, None
        self.rx, self.ry = None, None  # 记录最终的路径的x坐标和y坐标
        self.searched_points = []

    # 获取机器人的运动模型
    @staticmethod
    def get_motion_model():
        # [dx, dy, cost]
        # motion = [[1, 0, 1],  # 右
        #           [0, 1, 1],  # 上
        #           [-1, 0, 1],  # 左
        #           [0, -1, 1],  # 下
        #           [-1, -1, math.sqrt(2)],  # 左下
        #           [-1, 1, math.sqrt(2)],  # 左上
        #           [1, -1, math.sqrt(2)],  # 右下
        #           [1, 1, math.sqrt(2)]]  # 右上
        motion = [[1, 0, 1],  # 右
                  [0, 1, 1],  # 上
                  [-1, 0, 1],  # 左
                  [0, -1, 1],]  # 下
        return motion

    def inflate_obstacles(self, obstacle_map):
        # inflation_radius 是膨胀系数，表示除了障碍物本身外，额外膨胀的网格数量
        inflation_radius = int(self.inflation_radius)
        x_width, y_width = obstacle_map.shape[0], obstacle_map.shape[1]
        inflated_obstacle_map = obstacle_map.copy()  # 创建障碍物地图的副本以膨胀障碍物
        for ix in range(x_width):
            for iy in range(y_width):
                if obstacle_map[ix][iy] == True:  # 发现障碍物
                    # 遍历障碍物周围的网格，根据膨胀系数膨胀
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        for dy in range(-inflation_radius, inflation_radius + 1):
                            new_ix = ix + dx
                            new_iy = iy + dy
                            # 确保膨胀的网格在地图范围内
                            if 0 <= new_ix < x_width and 0 <= new_iy < y_width:
                                inflated_obstacle_map[new_ix][new_iy] = True
        return inflated_obstacle_map

    # 绘制栅格地图
    def calc_obstacle_map(self):
        # 设置x轴y轴边界
        self.min_x = 0
        self.min_y = 0
        self.max_x = self.original_obstacle_map.shape[0]
        self.max_y = self.original_obstacle_map.shape[1]

        # 地图的x和y方向的栅格个数，长度/每个网格的长度=网格个数
        self.x_width = round((self.max_x - self.min_x) / self.resolution)  # x方向网格个数
        self.y_width = round((self.max_y - self.min_y) / self.resolution)  # y方向网格个数

        # 初始化地图，二维列表，每个网格的值为False
        obstacle_res_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]

        # 设置障碍物
        for ix in range(self.x_width):  # 遍历x方向的网格 [0:x_width]
            x = self.calc_position(ix, self.min_x)  # 根据网格索引计算x坐标位置
            for iy in range(self.y_width):  # 遍历y方向的网格 [0:y_width]
                y = self.calc_position(iy, self.min_y)  # 根据网格索引计算y坐标位置
                if self.original_inflate_obstacle_map[round(x)][round(y)] == 1:  # 当前实际位置为障碍物
                    obstacle_res_map[ix][iy] = True

        return obstacle_res_map

    # 构建节点，每个网格代表一个节点
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # 网格索引
            self.y = y
            self.cost = cost  # 路径值
            self.parent_index = parent_index  # 该网格的父节点

        def __str__(self):
            return str(self.x) + ',' + str(self.y) + ',' + str(self.cost) + ',' + str(self.parent_index)

    # 寻找最优路径，网格起始坐标(sx,sy)，终点坐标（gx,gy）
    def planning(self, sx, sy, gx, gy):
        # 节点初始化 (sx,sy) 和（gx, gy）
        self.sx, self.sy = sx, sy
        self.gx, self.gy = gx, gy
        # 将已知的起点和终点坐标形式转化为节点类型，0代表路径权重，-1代表无父节点
        start_node = self.Node(self.calc_xy_index(self.sx, self.min_x),
                               self.calc_xy_index(self.sy, self.min_y), 0.0, -1)
        # 终点
        goal_node = self.Node(self.calc_xy_index(self.gx, self.min_x),
                              self.calc_xy_index(self.gy, self.min_y), 0.0, -1)
        # 保存入库节点和待计算节点
        open_set, closed_set = dict(), dict()
        # 先将起点入库，获取每个网格对应的key
        open_set[self.calc_index(start_node)] = start_node

        # 循环
        while 1:
            # 选择扩展点，添加了启发项，f(n)= g(n) + h(n)
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost + \
                                     self.calc_heuristic(goal_node, open_set[o], self.x_width, self.y_width))

            current = open_set[c_id]  # 从字典中取出该节点
            self.searched_points.append(current)  # 记录搜索过的节点

            # 判断是否是终点，如果选出来的损失最小的点是终点
            if current.x == goal_node.x and current.y == goal_node.y:
                # 更新终点的父节点
                goal_node.cost = current.cost
                # 更新终点的损失
                goal_node.parent_index = current.parent_index
                break

            # 在外库中删除该最小损失点，把它入库
            del open_set[c_id]
            closed_set[c_id] = current

            # 遍历邻接节点
            for move_x, move_y, move_cost in self.motion:
                # 获取每个邻接节点的节点坐标，到起点的距离，父节点
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                # 获取该邻居节点的key
                n_id = self.calc_index(node)

                # 如果该节点入库了，就看下一个
                if n_id in closed_set:
                    continue

                # 邻居节点是否超出范围了，是否在障碍物上
                if not self.verify_node(node):
                    continue

                # 如果该节点不在外库中，就作为一个新节点加入到外库
                if n_id not in open_set:
                    open_set[n_id] = node
                # 节点在外库中时
                else:
                    # 如果该点到起点的距离，要小于外库中该点的距离，就更新外库中的该点信息，更改路径
                    if node.cost <= open_set[n_id].cost:
                        open_set[n_id] = node

        # 找到终点
        self.rx, self.ry = self.calc_final_path(goal_node, closed_set)
        return self.rx, self.ry

    # 位置坐标转为网格坐标
    def calc_xy_index(self, position, minp):
        # (目标位置坐标-起点坐标)/一个网格的长度==>目标位置的网格索引
        return round((position - minp) / self.resolution)

    # 给每个网格编号，得到每个网格的key
    def calc_index(self, node):
        # 从左到右增大，从下到上增大
        return node.y * self.x_width + node.x

    # ------------------------------ #
    # A* 的启发函数
    # ------------------------------ #
    @staticmethod
    def calc_heuristic(goal, current, x_width, y_width):  # goal终点，current当前网格节点
        # w = 1.0  # 单个启发函数的权重，如果有多个启发函数，权重可以设置的不一样
        # d = w * math.hypot(goal.x - goal.x, current.y - current.y)  # 当前网格和终点距离
        # alpha, beta
        alpha, beta = 0.4, 0.6
        # d_manhattan
        d_m = abs(goal.x - current.x) + abs(goal.y - current.y)
        # d_Enclid
        d_e = math.hypot(goal.x - goal.x, current.y - current.y)
        # w_h
        l_c = x_width - current.x
        w_c = y_width - current.y
        # w = (1 + (d_m + d_e)/(l_c * w_c))
        w = 1 + (d_m + d_e)/(l_c * w_c)
        d = (alpha*d_m + beta*d_e)*w
        return d

    # 根据网格编号计算实际坐标
    def calc_position(self, index, minp):
        # minp代表起点坐标，左下x或左下y
        pos = minp + index * self.resolution  # 网格点左下左下坐标
        return pos

    # 判断该点是否在所框选区域内
    def is_in_poly(self, p, poly):
        """
        :param p: [x, y]
        :param poly: [[], [], [], [], ...]
        :return:
        """
        px, py = p
        is_in = False
        for i, corner in enumerate(poly):
            next_i = i + 1 if i + 1 < len(poly) else 0
            x1, y1 = corner
            x2, y2 = poly[next_i]
            if (x1 == px and y1 == py) or (x2 == px and y2 == py):  # if point is on vertex
                is_in = True
                break
            if min(y1, y2) < py <= max(y1, y2):  # find horizontal edges of polygon
                x = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
                if x == px:  # if point is on edge
                    is_in = True
                    break
                elif x > px:  # if point is on left-side of line
                    is_in = not is_in
        return is_in

    # 邻居节点是否超出范围
    def verify_node(self, node):
        # 根据网格坐标计算实际坐标
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        # 判断是否超出边界
        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False
        # # 不在所框选的区域内
        if not self.is_in_poly((px, py), self.ploy):
            return False
        # 节点是否在障碍物上，障碍物标记为True
        if self.obstacle_map[node.x][node.y]:
            return False
        # 没超过就返回True
        return True

    # 计算路径, parent属性记录每个节点的父节点
    def calc_final_path(self, goal_node, closed_set):
        # 先存放终点坐标（真实坐标）
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]
        # 获取终点的父节点索引
        parent_index = goal_node.parent_index
        # 起点的父节点==-1
        while parent_index != -1:
            n = closed_set[parent_index]  # 在入库中选择父节点
            rx.append(self.calc_position(n.x, self.min_x))  # 节点的x坐标
            ry.append(self.calc_position(n.y, self.min_y))  # 节点的y坐标
            parent_index = n.parent_index  # 节点的父节点索引

        simpler_points = [(x, y) for x, y in zip(rx, ry)]
        simpler_points = self.simplify_path(simpler_points)
        rx = [point[0] for point in simpler_points]
        ry = [point[1] for point in simpler_points]
        return rx, ry

    # 计算两点间的斜率，如果斜率不存在（即垂直），返回None
    @staticmethod
    def simplify_path(points):
        def slope(p1, p2):
            if p1[0] == p2[0]:  # x坐标相同，斜率不存在
                return None
            return (p2[1] - p1[1]) / (p2[0] - p1[0])

        simplified = [points[0]]  # 从起点开始
        i = 0
        while i < len(points) - 2:
            start = points[i]
            middle = points[i + 1]
            end = points[i + 2]

            # 计算斜率
            slope_start_middle = slope(start, middle)
            slope_middle_end = slope(middle, end)

            # 检查连续三点是否共线且斜率为0或不存在
            if slope_start_middle == slope_middle_end and (slope_start_middle == 0 or slope_start_middle is None):
                # 跳过中间点
                i += 1
                while i + 2 < len(points) and slope(points[i], points[i + 1]) == slope(points[i + 1],
                                                                                       points[i + 2]) and (
                        slope(points[i], points[i + 1]) == 0 or slope(points[i], points[i + 1]) is None):
                    i += 1
                simplified.append(points[i + 1])  # 添加共线的最后一个点
                i += 2  # 继续检查下一个点集
            else:
                simplified.append(middle)  # 如果不共线，添加中间点
                i += 1

        if simplified[-1] != points[-1]:  # 确保终点被添加
            simplified.append(points[-1])
        return simplified

    # 数组坐标系->图像坐标系或者图像坐标系->数组坐标系
    @staticmethod
    def convert_coordinates(x, y):
        return y, x

    # 绘画所有搜索过的(节点)、(路径)、(障碍物图像)
    def show_animation(self):
        plt.cla()
        plt.imshow(self.original_obstacle_map, cmap='gray')
        plt.plot(self.sy, self.sx, 'og')  # 起点绿色
        plt.plot(self.gy, self.gx, 'xb')  # 终点蓝色
        # # 显示所有搜索过的节点
        # for point in self.searched_points:
        #     # 网格索引转换为真实坐标
        #     plt.plot(self.calc_position(point.y, self.min_y),
        #              self.calc_position(point.x, self.min_x), 'xc')
        # 显示最终路径
        plt.plot(self.ry, self.rx, '-r')
        plt.show()


# 获取所有障碍物的坐标点
# def get_obstacle(obstacle_map):
#     ox, oy = [], []
#     for x in range(obstacle_map.shape[0]):
#         for y in range(obstacle_map.shape[1]):
#             if obstacle_map[x][y] == 1:
#                 ox.append(x)
#                 oy.append(y)
#     return ox, oy




