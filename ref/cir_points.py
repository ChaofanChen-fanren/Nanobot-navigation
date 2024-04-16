import matplotlib.pyplot as plt
from itertools import product

def points_in_circle(radius):
    for x, y in product(range(int(radius) + 1), repeat=2):
        if x**2 + y**2 <= radius**2:
            yield from set(((x, y), (x, -y), (-x, y), (-x, -y),))

# 获取圆形区域内的所有点
points = list(points_in_circle(2.5))

# 提取x坐标和y坐标
x_coords = [point[0] for point in points]
y_coords = [point[1] for point in points]

# 绘制点
plt.scatter(x_coords, y_coords, c='blue', label='Points in Circle')

# 设置图形标题和标签
plt.title('Points in Circle with Radius 2')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.gca().set_aspect('equal', adjustable='box')

# 显示图例
plt.legend()

# 显示图形
plt.show()
