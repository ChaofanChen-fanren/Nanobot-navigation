import numpy as np
import matplotlib.pyplot as plt


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

# def plot_circle_path(x, y):
#     plt.figure()
#     plt.plot(x, y, '.')
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('Circular Path')
#     plt.gca().set_aspect('equal', adjustable='box')
#     plt.show()
#
# # 设置起始点和半径
# x0, y0 = 100, 100
# radius = 100
# # 设置步长
# step_size = 15
# # 生成圆形路径
# x, y = generate_circle_path(x0, y0, radius, step_size)
# # 绘制路径
# plot_circle_path(x, y)



# # 设置起始点和边长
# x0, y0 = 100, 100
# side_length = 300
# # 设置步长
# step_size = 20
# # 生成正方形路径
# x, y = generate_square_path(x0, y0, side_length, step_size)
# # 绘制路径
# plot_square_path(x, y)


def plot_sin_path(x, y):
    plt.figure()
    plt.plot(x, y, '.')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Sin Path')
    plt.show()

# 设置起始点和幅度、周期
x0, y0 = 100, 100
amplitude = 210
period = 400
# 设置步长
step_size = 15
# 生成sin函数路径
x, y = generate_sin_path(x0, y0, amplitude, period, step_size)
# 绘制路径
plot_sin_path(x, y)