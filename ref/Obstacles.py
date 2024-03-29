import cv2
import numpy as np


# 使用opencv的findContours实现，效果不好
class Obstacles:
    def __init__(self):
        self.threshold1 = 30
        self.threshold2 = 50
        self.min_contour_area = 10000
        self.obstacles_xy = []
        self.obstacles_cnt = []

    @staticmethod
    def imgxy2robotxy(img_height, x, y):
        return x, img_height - y

    def contour2xy(self, frame, cnt):
        cnt_xy = [self.imgxy2robotxy(frame.shape[0], point[0][0], point[0][1]) for point in cnt]
        return np.array(cnt_xy)

    def get_obstacles_cnt(self):
        return self.obstacles_cnt

    def get_obstacles_xy(self):
        return self.obstacles_xy

    def find_all_obstacles(self, frame):
        # Convert frame to gray
        img_blur = cv2.GaussianBlur(frame, (7, 7), 1)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        # Canny edge detection
        img_canny = cv2.Canny(img_gray, self.threshold1, self.threshold2)

        # Dilate
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img_dil = cv2.dilate(img_canny, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(img_dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        list_xy = []
        list_cnt = []

        # Iterate through contours
        for cnt in contours:
            # If the contour area is greater than the minimum contour area
            if cv2.contourArea(cnt) > self.min_contour_area:
                # Add contour points to the lists in both image and robot coordinate systems
                list_xy.append(self.contour2xy(frame, cnt))
                list_cnt.append(cnt)

        # Update member variables with the results
        self.obstacles_xy = list_xy
        self.obstacles_cnt = list_cnt

        return list_xy

    def show_all_obstacles(self, frame):
        # Print all coordinates in obstacles_list_cnt (Img coordinates)
        for contour in self.obstacles_cnt:
            for point in contour:
                # print("------>" ,point);
                cv2.circle(frame, (point[0][0], point[0][1]), 5, (0, 0, 255), -1)


def test_obstacles():
    import time
    # Open camera. Parameter: camera_index
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)

    obstacles = Obstacles()

    # obstacles_list_xy = obstacles.get_obstacles_xy()
    # obstacles_list_cnt = obstacles.get_obstacles_cnt()

    # # Print all coordinates in obstacles_list_xy (Robot coordinates)
    # for contour in obstacles_list_xy:
    #     for point in contour:
    #         print(f"x: {point[0]}, y: {point[1]}")

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break
        try:
            obstacles.find_all_obstacles(frame)
            # Retrieve and print the current robot position
            obstacles.show_all_obstacles(frame)

            cv2.imshow("imgShow", frame)
            key = cv2.waitKey(10)
            if key == ord("q"):
                break
        except Exception as e:
            raise RuntimeError("Robot Obstacles Running Error: Error processing frame.") from e


if __name__ == '__main__':
    test_obstacles()
