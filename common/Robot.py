import cv2
from util.operation import YOLO

class Robot:
    def __init__(self, cap, frame_width, frame_height, img_frame=None, contours=None):
        self.robot_position = None
        # self.tracker = cv2.TrackerCSRT_create()
        params = cv2.TrackerVit_Params()
        params.net = "./common/object_tracking_vittrack_2023sep.onnx"
        self.tracker = cv2.TrackerVit_create(params)
        self.robot_img_position_list = list()

        # 检测粒子模型
        self.det_model = YOLO(onnx_path="./common/dect_robot.onnx")

        # Read the first frame
        self.cap = cap
        ret, frame = self.cap.read()
        self.frame_width, self.frame_height = frame_width, frame_height
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)

        if img_frame is not None:
            frame = cv2.addWeighted(frame, 0.1, img_frame, 0.9, 0)

        if contours is not None:
            cv2.drawContours(frame, contours, -1, (0, 0, 0), cv2.FILLED)

        if not ret:
            raise RuntimeError("Robot Constructor Error: Couldn't read the first frame.")
        self.fps = 0.0

        # Select ROI
        self.bbox = cv2.selectROI("Select ROI: Box Select the object you want to track", frame, True)
        cv2.destroyAllWindows()

        # Initialize tracker
        self.tracker.init(frame, self.bbox)

        # Initialize robot_position based on bbox center
        x = int(self.bbox[0] + self.bbox[2] / 2)
        y = int(self.bbox[1] + self.bbox[3] / 2)
        self.robot_position = self.imgxy2robotxy(frame.shape[0], x, y)

    @staticmethod
    def imgxy2robotxy(img_height, x, y):
        return x, img_height - y


    def det_init_tracker_bbox(self):
        ret, frame = self.cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        robot = self.det_model.decect(frame)
        is_update, robot_bbox, robot_conf = False, None, None
        for i in range(len(robot)):
            bbox_xyxy, conf = robot[i]['bbox'], robot[i]['conf']
            if conf > 0.75:
                is_update = True
                if robot_conf:
                    robot_bbox, robot_conf = bbox_xyxy, conf
                else:
                    if conf > robot_conf :
                        robot_bbox, robot_conf = bbox_xyxy, conf

        if is_update:
            print(f"重新更新追踪框，置信度为：{robot_conf}")

            def xyxy_to_xywh(box):
                # 上面函数的逆函数
                """Convert [x1 y1 x2 y2] box format to [x y w h] format."""
                x, y = min(box[0], box[2]), min(box[1], box[3])
                w, h = max(box[0], box[2]) - x, max(box[1], box[3]) - y
                return [x, y, w, h]
            self.tracker.init(frame, xyxy_to_xywh(robot_bbox))
            # Initialize robot_position based on bbox center
            x = int(self.bbox[0] + self.bbox[2] / 2)
            y = int(self.bbox[1] + self.bbox[3] / 2)
            self.robot_position = self.imgxy2robotxy(frame.shape[0], x, y)


    def init_tracker_bbox(self):
        ret, frame = self.cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        self.bbox = cv2.selectROI("Select ROI: Box Select the object you want to track", frame, True)
        cv2.destroyAllWindows()
        self.tracker.init(frame, self.bbox)

        # Initialize robot_position based on bbox center
        x = int(self.bbox[0] + self.bbox[2] / 2)
        y = int(self.bbox[1] + self.bbox[3] / 2)
        self.robot_position = self.imgxy2robotxy(frame.shape[0], x, y)

    def __del__(self):
        pass

    def get_robot_position(self):
        return self.robot_position

    def get_robot_bbox(self):
        return self.bbox

    def process_frame(self, frame):
        # Start timer
        timer = cv2.getTickCount()
        # Update tracker
        ok, self.bbox = self.tracker.update(frame)

        # Calculate Frames per second (FPS)
        self.fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

        # Update Robot Position
        if ok:
            x = int(self.bbox[0] + self.bbox[2] / 2)
            y = int(self.bbox[1] + self.bbox[3] / 2)
            self.robot_position = self.imgxy2robotxy(frame.shape[0], x, y)
            self.robot_img_position_list.append((x, y))
        else:
            # Tracking failure detected
            raise RuntimeError("Robot Tracking Running Error: Robot Tracking failure detected.")

    def show_robot_frame(self, frame, show_tracking_path):
        # pass
        # Display Tracking object bbox
        cv2.rectangle(frame, (int(self.bbox[0]), int(self.bbox[1])),
                      (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3])), (255, 0, 0), 2, 1)

        # # Display x y
        # cv2.putText(frame, "x : " + str(self.robot_position[0]), (100, 80),
        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (50, 170, 50), 5)
        #
        # cv2.putText(frame, "y : " + str(self.robot_position[1]), (100, 150),
        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (50, 170, 50), 5)
        if show_tracking_path:
            for position in self.robot_img_position_list:
                cv2.circle(frame, position, 5, (255, 0, 0), -1)



