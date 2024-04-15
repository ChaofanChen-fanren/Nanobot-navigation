import numpy as np

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

# 示例点
point1 = (1, 2)
point2 = (20, 66)
pixel_spacing = 10

# 生成坐标点
points = generate_points(point1, point2, pixel_spacing)

# 打印生成的坐标点
for point in points:
    print(point)
