import matplotlib.pyplot as plt
from Obstacle import Obstacle


def is_in_poly(p, poly):
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


ob = Obstacle(weights_path="./unet.pth", image_path='./1.jpg')
obstacle_map = ob.obstacle_map
# 示例
s = (379, 126)
g = (190, 350)  # 待判断的点

rect = [(40, 253), (297, 472), (531, 229), (261, 7)]  # 倾斜矩形的四个顶点，按顺序排列
ploy = rect
fig, ax = plt.subplots()  # 创建图形和坐标轴对象
polygon = plt.Polygon(ploy, color='red', alpha=0.8)  # 创建多边形对象
ax.add_patch(polygon)  # 将多边形添加到坐标轴上
ax.plot(s[0], s[1], 'bo')  # 绘制点，使用蓝色圆圈标记
ax.plot(g[0], g[1], 'go')  # 绘制点，使用蓝色圆圈标记
ax.imshow(obstacle_map, cmap='gray')


print(is_in_poly(s, rect))
print(is_in_poly(g, rect))
plt.show()

