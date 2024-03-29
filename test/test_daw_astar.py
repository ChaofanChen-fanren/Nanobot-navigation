from common import RobotControl, State, Obstacle
from util import distance
import matplotlib.pyplot as plt
import math
import numpy as np
import time

if __name__ == '__main__':
    # (467,88) (498,88)
    init_state = State(0, 350, 190, 0, math.pi / 3.0, 20.0 * math.pi / 180.0, 0, 0)
    obstacle = Obstacle(weights_path="../unet.pth", image_path='../image/1.jpg')
    robot = RobotControl(init_state, obstacle, [126, 379])

    cost = 0
    best_uv = np.array([0, 0])
    # 这里的for循环可以改成while循环，在船到达之前都要运动迭代
    for i in range(1000):
        time_begin = time.time()
        best_uv, cost = robot.search_for_best_uv()
        next_state = robot.motion(best_uv[0], best_uv[1], is_add=True)
        robot.states[-1].cost = cost
        print(robot.states[-1].x, robot.states[-1].y, robot.states[-1].yaw * 180 / math.pi, robot.states[-1].velocity)
        print(f"目标坐标{robot.goal}")
        # ship.obstacle_in_windows()
        time_end = time.time()
        # obstacle.update()
        # ship.initialobstacle(obstacle)
        # time.sleep(1)
        if robot.check_collision():
            break
        print("第%d次迭代，耗时%.6fs，当前距离终点%.6f" % (
            i + 1, (time_end - time_begin), distance(np.array([robot.states[-1].x, robot.states[-1].y]), robot.goal)))
        if distance(np.array([robot.states[-1].x, robot.states[-1].y]), robot.goal) < 10:
            if not robot.update_goal():
                print("Done!")
                break
            else:
                print(f"-----更新目标点坐标为{robot.goal}")
        # print("当前轨迹值：")
        # for i in ship.ship:
        #     print("(%.2f,%.2f)"%(i.x,i.y))
    # fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15, 10))
    #
    # ax[0].scatter(ship.obstacle[:, 0], ship.obstacle[:, 1], s=5)
    # ax[0].plot(ship.goal[0], ship.goal[1], "ro")
    # lx = []
    # ly = []
    # lt = []
    # lc = []
    # for i in ship.ship:
    #     lx.append(i.x)
    #     ly.append(i.y)
    #     lt.append(i.time)
    #     lc.append(i.cost)
    # ax[0].plot(lx, ly)
    # for i in range(17):
    #     locusx, locusy = obstacle.returnlocus(i)
    #     ax[0].plot(locusx, locusy)
    # ax[0].grid(True)
    # ax[0].set_title(label="locus figure")
    # ax[1].scatter(lt, lc, s=2)
    # ax[1].grid(True)
    # ax[1].set_title("cost figure")

    # plt.show()
