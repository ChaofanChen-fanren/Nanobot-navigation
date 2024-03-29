import torch
from .Unet import UNet
import numpy as np
from torchvision import transforms
import cv2


"""
障碍物类：Obstacle
obstacle_map：存储二维平面的障碍物，0代表无障碍物，1代表有障碍物
obstacle_map_list：每一次更新后将之前的障碍物地图保存
"""


class Obstacle(object):
    def __init__(self, weights_path, image_path):
        # segment model device
        self.device = "cpu"
        print("using {} device.".format(self.device))

        classes = 1  # exclude background
        # create model
        self.model = UNet(in_channels=3, num_classes=classes + 1, base_c=32)
        # load weights
        self.model.load_state_dict(torch.load(weights_path, map_location='cpu')['model'])
        self.model.to(self.device)

        mean = (0.709, 0.381, 0.224)
        std = (0.127, 0.079, 0.043)
        # from pil image to tensor and normalize
        self.data_transform = transforms.Compose([transforms.ToTensor(),
                                                  transforms.Normalize(mean=mean, std=std)])

        # 识别图像将障碍物转化为0和1
        self.img_path = image_path
        self.obstacle_map = self.process_image(self.img_path)
        # self.assemble = 3 * np.array([
        #     [-1, -1],
        #     [0, 2],
        #     [4.0, 2.0],
        #     [4.0, 1.0],
        #     [5.0, 4.0],
        #     [2.5, 4.0],
        #     [5.0, 5.0],
        #     [5.0, 2.5],
        #     [5.0, 6.0],
        #     [5.0, 9.0],
        #     [6.0, 6.0],
        #     [7.0, 6.0],
        #     [10.0, 8.0],
        #     [10.0, 4.0],
        #     [8.0, 9.0],
        #     [7.0, 9.0],
        #     [12.0, 12.0]
        # ])
        self.obstacle_map_list = [self.obstacle_map, ]  # 记录所有的地图信息（有更新函数）

    # 更新障碍物地图
    def update(self, img_path):
        self.img_path = img_path
        self.obstacle_map = self.process_image(img_path)
        self.obstacle_map_list = self.obstacle_map_list.append(self.obstacle_map)

    def get_current_obstacle_map(self):
        return self.obstacle_map

    # 获取所有障碍物的坐标点
    def get_obstacle_points(self):
        obstacle_points = []
        for x in range(self.obstacle_map.shape[0]):
            for y in range(self.obstacle_map.shape[1]):
                if self.obstacle_map[x][y] == 1:
                    obstacle_points.append([x, y])
        print(f"障碍物的总数为: {len(obstacle_points)}")
        return np.array(obstacle_points)

    def process_image(self, image_path):
        # load image
        original_img = cv2.imread(image_path)
        img = self.data_transform(original_img)
        # expand batch dimension
        image_bgr = torch.unsqueeze(img, dim=0)
        # 语义分割预测
        self.model.eval()  # 进入验证模式
        with torch.no_grad():
            # init model
            img_height, img_width = image_bgr.shape[-2:]
            init_img = torch.zeros((1, 3, img_height, img_width), device=self.device)
            self.model(init_img)
            output = self.model(image_bgr.to(self.device))
            prediction = output['out'].argmax(1).squeeze(0)
            pred_mask = prediction.to("cpu").numpy().astype(np.uint8)
        kernel = np.ones((7, 7), np.uint8)

        # 对图像进行腐蚀操作
        pred_mask = cv2.erode(pred_mask, kernel, iterations=1)
        return pred_mask

    def show_viz_img(self):
        image_bgr = cv2.imread(self.img_path)
        pred_mask = self.obstacle_map
        # 各类别的配色方案（BGR）
        palette = [
            ['background', [127, 127, 127]],
            ['obstruction', [0, 0, 255]],
        ]

        palette_dict = {}
        for idx, each in enumerate(palette):
            palette_dict[idx] = each[1]

        # 将预测的整数ID，映射为对应类别的颜色
        pred_mask_bgr = np.zeros((pred_mask.shape[0], pred_mask.shape[1], 3))
        for idx in palette_dict.keys():
            pred_mask_bgr[np.where(pred_mask == idx)] = palette_dict[idx]
        pred_mask_bgr = pred_mask_bgr.astype('uint8')

        # 将语义分割预测图和原图叠加显示
        opacity = 0.5  # 透明度
        pred_viz = cv2.addWeighted(image_bgr, opacity, pred_mask_bgr, 1 - opacity, 0)
        return pred_viz





