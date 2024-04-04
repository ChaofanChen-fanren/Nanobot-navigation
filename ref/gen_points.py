import cv2
import numpy as np

def calculate_red_density(image, block_size):
    # 将图像转换为 HSV 颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 提取红色区域
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([160, 100, 100])
    upper_red = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    mask = cv2.bitwise_or(mask1, mask2)

    # 计算每个块内的红色像素密度
    height, width = mask.shape[:2]
    density_map = np.zeros((height // block_size, width // block_size), dtype=np.float32)

    for i in range(0, height - block_size + 1, block_size):
        for j in range(0, width - block_size + 1, block_size):
            block = mask[i:i+block_size, j:j+block_size]
            density_map[i // block_size, j // block_size] = np.mean(block)

    return density_map

def identify_thrombus(density_map, density_threshold):
    # 标记高密度区域为血栓区域
    thrombus_mask = np.zeros_like(density_map, dtype=np.uint8)
    thrombus_mask[density_map > density_threshold] = 255

    return thrombus_mask

# 读取图像
image = cv2.imread('../image/cell.png')

# 计算红色像素密度
block_size = 5  # 设置块大小，可以根据图像尺寸和红色密度的预期范围进行调整
red_density_map = calculate_red_density(image, block_size)

# 识别血栓区域
density_threshold = 5 # 设置红色密度阈值，可以根据实际情况进行调整
thrombus_mask = identify_thrombus(red_density_map, density_threshold)

# 显示结果
cv2.imshow('Original Image', image)
cv2.imshow('Red Density Map', (red_density_map / np.max(red_density_map) * 255).astype(np.uint8))
thrombus_mask = cv2.resize(thrombus_mask,(1920, 1080))
cv2.imshow('Thrombus Mask', thrombus_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()


# 读取图像
# image = cv2.imread('../image/cell.png')



