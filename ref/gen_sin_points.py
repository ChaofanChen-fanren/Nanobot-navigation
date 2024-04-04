import numpy as np
import matplotlib.pyplot as plt

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

# 示例点
point1 = (1, 2)
point2 = (150, 200)
pixel_spacing = 2

# 生成坐标点
points = generate_sin_wave(point1, point2, pixel_spacing)

# 绘图
plt.figure(figsize=(8, 6))
plt.plot(points[:, 0], points[:, 1], marker='o', linestyle='-')
plt.title('Sin Wave along the Line')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()
