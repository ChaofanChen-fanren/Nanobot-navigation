def simplify_path(points):
    # 计算两点间的斜率，如果斜率不存在（即垂直），返回None
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
            while i + 2 < len(points) and slope(points[i], points[i + 1]) == slope(points[i + 1], points[i + 2]) and (
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


# # 示例使用
# points = [(0, 0), (1, 0), (2, 0), (3, 3), (4, 4), (5, 5), (6, 5), (7, 5)]
# simplified_points = simplify_path(points)
# print(simplified_points)
