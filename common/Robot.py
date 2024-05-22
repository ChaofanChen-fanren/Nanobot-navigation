import cv2


class Robot:
    def __init__(self, cap, frame_width, frame_height, img_frame=None, contours=None):
        self.robot_position = None
        self.tracker = cv2.TrackerCSRT_create()
        self.robot_img_position_list = list()

        # Read the first frame
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (frame_width, frame_height))
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

    def show_robot_frame(self, frame):
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

        # for position in self.robot_img_position_list:
        #     cv2.circle(frame, position, 5, (255, 0, 0), -1)



