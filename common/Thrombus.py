import torch
from Unet import UNet
import numpy as np
from torchvision import transforms
import cv2
from enum import IntEnum
from util import GridMap, FloatGrid, rot_mat_2d
import math


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 ox, oy, resolution,
                 moving_direction=MovingDirection.RIGHT,
                 sweep_direction=SweepDirection.UP
                 ):

        self.sweep_vec, self.sweep_start_position = None, None
        self.ox, self.oy = ox, oy
        self.resolution = resolution
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        grid_map, x_inds_goal_y, goal_y = self.pre_process()

        self.turing_window = []
        self.update_turning_window()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y
        self.grid_map = grid_map

        px, py = self.sweep_path_search()
        self.rx, self.ry = self.convert_global_coordinate(px, py, self.sweep_vec,
                                                          self.sweep_start_position)

    def pre_process(self):
        self.sweep_vec, self.sweep_start_position = self.find_sweep_direction_and_start_position(
            self.ox, self.oy)
        rox, roy = self.convert_grid_coordinate(self.ox, self.oy, self.sweep_vec,
                                                self.sweep_start_position)
        grid_map, x_inds_goal_y, goal_y = self.setup_grid_map(rox, roy, self.resolution,
                                                              self.sweep_direction)
        return grid_map, x_inds_goal_y, goal_y

    def get_res(self):
        return self.rx, self.ry

    def sweep_path_search(self):
        # search start grid
        c_x_index, c_y_index = self.search_start_grid(self.grid_map)
        if not self.grid_map.set_value_from_xy_index(c_x_index, c_y_index, FloatGrid(0.5)):
            print("Cannot find start grid")
            return [], []

        x, y = self.grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index,
                                                                    c_y_index)
        px, py = [x], [y]

        while True:
            c_x_index, c_y_index = self.move_target_grid(c_x_index,
                                                         c_y_index,
                                                         self.grid_map)

            if self.is_search_done(self.grid_map) or (
                    c_x_index is None or c_y_index is None):
                print("Done")
                break

            x, y = self.grid_map.calc_grid_central_xy_position_from_xy_index(
                c_x_index, c_y_index)

            px.append(x)
            py.append(y)

            self.grid_map.set_value_from_xy_index(c_x_index, c_y_index, FloatGrid(0.5))

        return px, py

    def move_target_grid(self, c_x_index, c_y_index, grid_map):
        n_x_index = self.moving_direction + c_x_index
        n_y_index = c_y_index

        # found safe grid
        if not self.check_occupied(n_x_index, n_y_index, grid_map):
            return n_x_index, n_y_index
        else:  # occupied
            next_c_x_index, next_c_y_index = self.find_safe_turning_grid(
                c_x_index, c_y_index, grid_map)
            if (next_c_x_index is None) and (next_c_y_index is None):
                # moving backward
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if self.check_occupied(next_c_x_index, next_c_y_index, grid_map, FloatGrid(1.0)):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not self.check_occupied(next_c_x_index + self.moving_direction, next_c_y_index, grid_map):
                    next_c_x_index += self.moving_direction
                self.swap_moving_direction()
            return next_c_x_index, next_c_y_index

    @staticmethod
    def check_occupied(c_x_index, c_y_index, grid_map, occupied_val=FloatGrid(0.5)):
        return grid_map.check_occupied_from_xy_index(c_x_index, c_y_index, occupied_val)

    @staticmethod
    def find_sweep_direction_and_start_position(ox, oy):
        # find sweep_direction
        max_dist = 0.0
        vec = [0.0, 0.0]
        sweep_start_pos = [0.0, 0.0]
        for i in range(len(ox) - 1):
            dx = ox[i + 1] - ox[i]
            dy = oy[i + 1] - oy[i]
            d = np.hypot(dx, dy)

            if d > max_dist:
                max_dist = d
                vec = [dx, dy]
                sweep_start_pos = [ox[i], oy[i]]

        return vec, sweep_start_pos

    @staticmethod
    def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_position):
        tx = [ix - sweep_start_position[0] for ix in ox]
        ty = [iy - sweep_start_position[1] for iy in oy]
        th = math.atan2(sweep_vec[1], sweep_vec[0])
        converted_xy = np.stack([tx, ty]).T @ rot_mat_2d(th)

        return converted_xy[:, 0], converted_xy[:, 1]

    @staticmethod
    def convert_global_coordinate(x, y, sweep_vec, sweep_start_position):
        th = math.atan2(sweep_vec[1], sweep_vec[0])
        converted_xy = np.stack([x, y]).T @ rot_mat_2d(-th)
        rx = [ix + sweep_start_position[0] for ix in converted_xy[:, 0]]
        ry = [iy + sweep_start_position[1] for iy in converted_xy[:, 1]]
        return rx, ry

    def setup_grid_map(self, ox, oy, resolution, sweep_direction, offset_grid=10):
        width = math.ceil((max(ox) - min(ox)) / resolution) + offset_grid
        height = math.ceil((max(oy) - min(oy)) / resolution) + offset_grid
        center_x = (np.max(ox) + np.min(ox)) / 2.0
        center_y = (np.max(oy) + np.min(oy)) / 2.0

        grid_map = GridMap(width, height, resolution, center_x, center_y)
        # grid_map.print_grid_map_info()
        grid_map.set_value_from_polygon(ox, oy, FloatGrid(1.0), inside=False)
        grid_map.expand_grid()

        x_inds_goal_y = []
        goal_y = 0
        if sweep_direction == SweepSearcher.SweepDirection.UP:
            x_inds_goal_y, goal_y = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
            x_inds_goal_y, goal_y = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        return grid_map, x_inds_goal_y, goal_y

    @staticmethod
    def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
        y_index = None
        x_indexes = []

        if from_upper:
            x_range = range(grid_map.height)[::-1]
            y_range = range(grid_map.width)[::-1]
        else:
            x_range = range(grid_map.height)
            y_range = range(grid_map.width)

        for iy in x_range:
            for ix in y_range:
                if not SweepSearcher.check_occupied(ix, iy, grid_map):
                    y_index = iy
                    x_indexes.append(ix)
            if y_index:
                break

        return x_indexes, y_index

    def find_safe_turning_grid(self, c_x_index, c_y_index, grid_map):

        for (d_x_ind, d_y_ind) in self.turing_window:

            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not self.check_occupied(next_x_ind, next_y_ind, grid_map):
                return next_x_ind, next_y_ind

        return None, None

    def is_search_done(self, grid_map):
        for ix in self.x_indexes_goal_y:
            if not self.check_occupied(ix, self.goal_y, grid_map):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        # turning window definition
        # robot can move grid based on it.
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")


class Thrombus(object):
    """
    血栓类：Obstacle
    obstacle_map：存储二维平面的障碍物，0代表无障碍物，1代表有障碍物
    obstacle_map_list：每一次更新后将之前的障碍物地图保存
    """

    def __init__(self, weights_path='./best_thrombus_model.pth', image_path=None, frame=None):
        # segment model device

        self.device = "cpu"
        print("using {} device.".format(self.device))

        classes = 1  # exclude background
        # create model
        self.model = UNet(in_channels=3, num_classes=classes + 1, base_c=32, bilinear=False)
        # load weights
        self.model.load_state_dict(torch.load(weights_path, map_location='cpu')['model'])
        self.model.to(self.device)

        mean = (0.541, 0.601, 0.692)
        std = (0.096, 0.108, 0.142)
        # from pil image to tensor and normalize
        self.data_transform = transforms.Compose([transforms.ToTensor(),
                                                  transforms.Normalize(mean=mean, std=std)])

        # 识别图像将障碍物转化为0和1
        if image_path:
            self.img_path = image_path
            self.frame = cv2.imread(self.img_path)
        else:
            self.frame = frame
        self.thrombus_ploy = self.process_image()

        # 覆盖算法
        self.ox, self.oy = None, None
        self.resolution = 30
        self.px, self.py = self.get_cover_planning()

    """
    返回最大面积的血栓
    """

    def get_thrombus_ploy(self, pred_mask):
        imgGray = pred_mask
        # kernel = np.ones((7, 7))
        # imgGray = cv2.dilate(imgGray, kernel, iterations=20)
        contours, hierarchy = cv2.findContours(imgGray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        ploy = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # areaMin = cv2.getTrackbarPos("Area", "Parameters")
            areaMin = 10000
            if area > areaMin:
                # 识别是几边形状
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
                ploy.append(approx.reshape(-1, 2))
                print(f"血栓的形状为：{len(approx)}多边形")

        return ploy[0]

    def process_image(self):
        # load image
        original_img = self.frame
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
        np.expand_dims(pred_mask * 255, axis=2)

        ploy = self.get_thrombus_ploy(pred_mask)

        return ploy

    def get_cover_planning(self):
        self.ox, self.oy = [point[0] for point in self.thrombus_ploy], [point[1] for point in self.thrombus_ploy]
        self.ox.append(self.thrombus_ploy[0][0])
        self.oy.append(self.thrombus_ploy[0][1])
        sweep = SweepSearcher(self.ox, self.oy, self.resolution)
        px, py = sweep.get_res()
        return px, py

    def show_thrombus_shape(self):
        show_frame = self.frame.copy()
        show_frame = cv2.polylines(show_frame, [self.thrombus_ploy],
                                   isClosed=True, color=(0, 0, 255), thickness=10)
        cv2.imshow("thrombus_ploy", show_frame)
        cv2.waitKey(0)

    def get_px_py(self):
        return self.px, self.py

    def show_animation(self):
        import matplotlib.pyplot as plt
        px, py = self.px, self.py

        for ipx, ipy in zip(px, py):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.imshow(self.frame[:, :, ::-1])
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(self.ox, self.oy, "-xb")
            plt.plot(px, py, "-r")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            # plt.grid(True)
            plt.pause(0.1)

        plt.cla()
        plt.plot(self.ox, self.oy, "-xb")
        plt.plot(px, py, "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)
        plt.close()


if __name__ == "__main__":
    thrombus = Thrombus(weights_path="../best_thrombus_model.pth", image_path="../image/130.jpg")
    thrombus.show_animation()
