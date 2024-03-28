import cv2


class Robot:
    def __init__(self, cap):
        self.robot_position = None
        self.tracker = cv2.TrackerCSRT_create()
        self.robot_img_position_list = list()

        # Read the first frame
        ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (1920, 1080))
        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
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
        # Display Tracking object bbox
        cv2.rectangle(frame, (int(self.bbox[0]), int(self.bbox[1])),
                      (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3])), (255, 0, 0), 2, 1)
        # # Display x y
        # cv2.putText(frame, "x : " + str(self.robot_position[0]), (100, 80),
        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (50, 170, 50), 5)
        #
        # cv2.putText(frame, "y : " + str(self.robot_position[1]), (100, 150),
        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (50, 170, 50), 5)

        for position in self.robot_img_position_list:
            cv2.circle(frame, position, 5, (255, 0, 0), -1)


def test_one_robot():
    import time
    # Open camera. Parameter: camera_index
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)

    # Parameter: camera_index
    robot = Robot(cap)
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            cv2.waitKey(10)
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


def test_multiple_robot():
    import time
    # Open camera. Parameter: camera_index
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)

    # Parameter: camera_index
    robot = Robot(cap)
    robot1 = Robot(cap)

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            robot1.process_frame(frame)
            robot1.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            robot_position1 = robot1.get_robot_position()
            print(f"Robot Position: {robot_position1[0]}  {robot_position1[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            key = cv2.waitKey(10)
            if key == ord("q"):
                break
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


if __name__ == '__main__':
    # Example usage:
    # Uncomment the test case you want to run
    import cv2
    test_one_robot()
    # test_multiple_robot()
